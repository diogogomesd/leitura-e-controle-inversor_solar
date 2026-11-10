#!/usr/bin/env python3
"""
leitor_inversor.py - Raspberry Pi 3

Lê o inversor Anenji 6200W pela RS232 (protocolo Voltronic/PI30, comando QPIGS),
calcula se a energia das placas é suficiente para os ar-condicionados e publica
a decisão no broker MQTT local (Mosquitto no próprio Pi).

Tópicos publicados:
  controle_solar/decisao             "SUFICIENTE" ou "INSUFICIENTE" (a cada 5 s, não retido)
  controle_solar/inversor            JSON com os dados do inversor e os cálculos
  controle_solar/raspberry/status    "online" / "offline" (retido, com Last Will)

Como decide:
  - corrente necessária = (QTD_AR x POTENCIA_AR_W) / tensão da saída AC
  - corrente das placas = (potência PV x EFICIENCIA) / tensão da saída AC
  - INSUFICIENTE -> SUFICIENTE: placas cobrem a necessária + MARGEM por TEMPO_CONFIRMA_SUFICIENTE
  - SUFICIENTE -> INSUFICIENTE: bateria descarregando >= DESCARGA_MAX_A por TEMPO_CONFIRMA_INSUFICIENTE,
    bateria abaixo de BATERIA_TENSAO_MIN (na hora) ou sem dados do inversor por TEMPO_SEM_DADOS.

Ligação pela UART dos pinos GPIO (obrigatório usar conversor MAX3232, a RS232 trabalha com ±12 V
e os pinos do Pi são de 3,3 V):
  Pino 1  (3V3)          -> MAX3232 VCC   (alimente o MAX3232 com 3,3 V, nunca 5 V)
  Pino 6  (GND)          -> MAX3232 GND
  Pino 8  (GPIO14 / TXD) -> MAX3232 RXD (lado TTL)
  Pino 10 (GPIO15 / RXD) <- MAX3232 TXD (lado TTL)
  MAX3232 lado RS232     <-> RJ45 do inversor pinos 1/2, pino 8 = GND (se não responder, inverta 1 e 2)

Preparar o Pi 3 (uma vez só):
  1. sudo raspi-config -> Interface Options -> Serial Port
       "login shell over serial?" -> Não     "serial port hardware enabled?" -> Sim
  2. Em /boot/firmware/config.txt (ou /boot/config.txt em sistemas antigos), adicionar:
       enable_uart=1
       dtoverlay=disable-bt
     (no Pi 3 a UART boa, PL011, fica com o Bluetooth; isso a devolve para os pinos 8 e 10)
  3. sudo systemctl disable hciuart
  4. sudo reboot   ->   ls -l /dev/serial0 deve apontar para ttyAMA0

Dependências:  sudo apt install python3-serial python3-paho-mqtt
"""

import json
import logging
import os
import signal
import sys
import time

import paho.mqtt.client as mqtt
import serial

# ---------- Serial do inversor ----------
PORTA_SERIAL = "/dev/serial0"  # UART dos pinos 8 (TX) e 10 (RX)
BAUD = 2400
TIMEOUT_INVERSOR = 2.0  # s

# ---------- MQTT ----------
MQTT_HOST = "localhost"
MQTT_PORT = 1883
MQTT_USER = "solar"
MQTT_PASSWORD = "troque_esta_senha"

TOPIC_DECISAO = "controle_solar/decisao"
TOPIC_INVERSOR = "controle_solar/inversor"
TOPIC_STATUS = "controle_solar/raspberry/status"

# ---------- Cargas (ajuste para os seus aparelhos) ----------
QTD_AR = 2               # quantidade de ar-condicionados
POTENCIA_AR_W = 1100     # consumo de cada um em W (veja a etiqueta; 12000 BTU ~ 1100 W)
EFICIENCIA = 0.90        # perdas do inversor (PV -> saída AC)
MARGEM = 0.15            # folga exigida para considerar suficiente (15%)
TENSAO_AC_PADRAO = 220.0 # usada se o inversor não informar a tensão de saída

# ---------- Bateria ----------
DESCARGA_MAX_A = 10        # descarga da bateria (A) que indica placas insuficientes
BATERIA_TENSAO_MIN = 48.0  # abaixo disso volta para INSUFICIENTE na hora

# ---------- Temporização (s) ----------
INTERVALO = 5
TEMPO_CONFIRMA_SUFICIENTE = 60
TEMPO_CONFIRMA_INSUFICIENTE = 30
TEMPO_SEM_DADOS = 30

DEBUG_INVERSOR = True  # mostra a resposta bruta do inversor

log = logging.getLogger("controle_solar")


# ================= Verificação da UART do Pi =================

def verificar_uart():
    """Avisa sobre configurações do Pi que impedem a leitura pelos pinos 8 e 10."""
    if not os.path.exists(PORTA_SERIAL):
        log.error("%s não existe. Ative a serial no raspi-config e adicione enable_uart=1 "
                  "no config.txt.", PORTA_SERIAL)
        return

    destino = os.path.basename(os.path.realpath(PORTA_SERIAL))
    log.info("%s -> %s", PORTA_SERIAL, destino)
    if destino == "ttyS0":
        log.warning("%s está na mini UART (ttyS0), que perde a velocidade quando o clock da CPU muda. "
                    "Adicione dtoverlay=disable-bt no config.txt para usar a ttyAMA0.", PORTA_SERIAL)

    try:
        with open("/proc/cmdline") as f:
            cmdline = f.read()
    except OSError:
        return
    if any(f"console={nome}" in cmdline for nome in ("serial0", "ttyAMA0", "ttyS0")):
        log.error("O console do Linux está usando a serial e vai atrapalhar o inversor. "
                  "No raspi-config, desative o 'login shell over serial'.")


# ================= Inversor (protocolo Voltronic / PI30) =================

def crc_voltronic(dados: bytes) -> bytes:
    """CRC-16/XMODEM com o ajuste do protocolo Voltronic (evita '(', '\\r' e '\\n' no CRC)."""
    crc = 0
    for b in dados:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    hi, lo = crc >> 8, crc & 0xFF
    if lo in (0x28, 0x0D, 0x0A):
        lo += 1
    if hi in (0x28, 0x0D, 0x0A):
        hi += 1
    return bytes([hi, lo])


def consultar(ser: serial.Serial, comando: str) -> str:
    """Envia um comando e devolve o texto da resposta (sem '(' e sem CRC)."""
    dados = comando.encode("ascii")
    ser.reset_input_buffer()
    ser.write(dados + crc_voltronic(dados) + b"\r")

    resp = ser.read_until(b"\r")
    if DEBUG_INVERSOR:
        log.debug("Resposta bruta: %r", resp)
    if not resp.endswith(b"\r"):
        raise TimeoutError("inversor sem resposta (verifique cabo, MAX3232 e pinos 1/2 do RJ45)")

    resp = resp[:-1]
    if len(resp) < 10 or resp[:1] != b"(":
        raise ValueError("resposta inválida")
    if crc_voltronic(resp[:-2]) != resp[-2:]:
        raise ValueError("CRC inválido")

    texto = resp[1:-2].decode("ascii", errors="replace")
    if texto.startswith("NAK"):
        raise ValueError("inversor respondeu NAK (comando não reconhecido)")
    return texto


def ler_qpigs(ser: serial.Serial) -> dict:
    """
    Resposta do QPIGS:
    (BBB.B CC.C DDD.D EE.E FFFF GGGG HHH III JJ.JJ KKK OOO TTTT EEEE UUU.U WW.WW PPPPP b7..b0 QQ VV MMMMM b10b9b8
    """
    c = consultar(ser, "QPIGS").split()
    if len(c) < 17:
        raise ValueError(f"poucos campos ({len(c)})")

    pv_tensao = float(c[13])
    pv_corrente = float(c[12])
    return {
        "rede_tensao": float(c[0]),
        "rede_freq": float(c[1]),
        "saida_tensao": float(c[2]),
        "saida_freq": float(c[3]),
        "saida_va": int(c[4]),
        "saida_w": int(c[5]),
        "carga_pct": int(c[6]),
        "bateria_tensao": float(c[8]),
        "bateria_carga_a": int(c[9]),
        "bateria_pct": int(c[10]),
        "temperatura": int(c[11]),
        "pv_corrente": pv_corrente,
        "pv_tensao": pv_tensao,
        "bateria_descarga_a": int(c[15]),
        # Campo 19 (potência PV) só existe em alguns firmwares; senão calcula V x I
        "pv_potencia": int(c[19]) if len(c) > 19 else int(pv_tensao * pv_corrente),
    }


# ================= Decisão =================

class Decisor:
    def __init__(self):
        self.decisao = "INSUFICIENTE"  # começa sempre pelo modo seguro
        self._condicao_desde = None

    def _confirmado(self, condicao: bool, tempo: float) -> bool:
        """True quando a condição se mantém verdadeira por 'tempo' segundos seguidos."""
        if not condicao:
            self._condicao_desde = None
            return False
        if self._condicao_desde is None:
            self._condicao_desde = time.monotonic()
        return time.monotonic() - self._condicao_desde >= tempo

    def mudar(self, nova: str, motivo: str):
        if nova == self.decisao:
            return
        log.info(">>> Decisão: %s -> %s (%s)", self.decisao, nova, motivo)
        self.decisao = nova
        self._condicao_desde = None

    def avaliar(self, d: dict) -> dict:
        tensao_ac = d["saida_tensao"] if d["saida_tensao"] > 100 else TENSAO_AC_PADRAO
        necessaria_w = QTD_AR * POTENCIA_AR_W
        disponivel_w = d["pv_potencia"] * EFICIENCIA

        calc = {
            "tensao_ac": round(tensao_ac, 1),
            "necessaria_w": necessaria_w,
            "necessaria_a": round(necessaria_w / tensao_ac, 2),
            "disponivel_w": round(disponivel_w),
            "disponivel_a": round(disponivel_w / tensao_ac, 2),
        }

        log.info("AC saída: %.1f V %d W | Necessário: %d W (%.1f A) | Placas: %.0f W (%.1f A) | "
                 "Bat: %.2f V %d%% desc %d A",
                 tensao_ac, d["saida_w"], necessaria_w, calc["necessaria_a"],
                 disponivel_w, calc["disponivel_a"],
                 d["bateria_tensao"], d["bateria_pct"], d["bateria_descarga_a"])

        if self.decisao == "INSUFICIENTE":
            sobra = disponivel_w >= necessaria_w * (1 + MARGEM)
            if self._confirmado(sobra, TEMPO_CONFIRMA_SUFICIENTE):
                self.mudar("SUFICIENTE", "placas cobrem o consumo com margem")
        else:
            if 0 < d["bateria_tensao"] <= BATERIA_TENSAO_MIN:
                self.mudar("INSUFICIENTE", "bateria baixa")
            elif self._confirmado(d["bateria_descarga_a"] >= DESCARGA_MAX_A, TEMPO_CONFIRMA_INSUFICIENTE):
                self.mudar("INSUFICIENTE", "bateria descarregando")

        return calc


# ================= MQTT =================

def ao_conectar(cliente, userdata, flags, rc, properties=None):
    if rc == 0:
        log.info("Conectado ao broker MQTT.")
        cliente.publish(TOPIC_STATUS, "online", qos=1, retain=True)
    else:
        log.error("Falha ao conectar no broker: %s", rc)


def criar_cliente() -> mqtt.Client:
    try:  # paho-mqtt 2.x
        cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="raspberry-inversor")
    except AttributeError:  # paho-mqtt 1.x
        cliente = mqtt.Client(client_id="raspberry-inversor")
    cliente.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    # Se o Pi cair, o broker avisa o ESP32 publicando "offline"
    cliente.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)
    cliente.on_connect = ao_conectar
    cliente.reconnect_delay_set(min_delay=1, max_delay=10)
    return cliente


# ================= Principal =================

def main():
    logging.basicConfig(level=logging.DEBUG if DEBUG_INVERSOR else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    # systemd encerra com SIGTERM: transforma em saída normal para executar o finally
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    verificar_uart()

    cliente = criar_cliente()
    cliente.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
    cliente.loop_start()  # thread em segundo plano, reconecta sozinha

    decisor = Decisor()
    ser = None
    ultima_leitura_valida = time.monotonic()

    try:
        while True:
            inicio = time.monotonic()
            try:
                if ser is None:
                    ser = serial.Serial(PORTA_SERIAL, BAUD, timeout=TIMEOUT_INVERSOR)
                    log.info("Serial %s aberta.", PORTA_SERIAL)

                dados = ler_qpigs(ser)
                ultima_leitura_valida = time.monotonic()
                calc = decisor.avaliar(dados)

                if cliente.is_connected():
                    cliente.publish(TOPIC_INVERSOR,
                                    json.dumps({**dados, **calc, "decisao": decisor.decisao}),
                                    qos=0)
            except serial.SerialException as e:
                log.error("Erro na serial: %s", e)
                if ser is not None:
                    ser.close()
                ser = None
            except (TimeoutError, ValueError) as e:
                log.warning("Inversor: %s", e)

            if time.monotonic() - ultima_leitura_valida > TEMPO_SEM_DADOS:
                decisor.mudar("INSUFICIENTE", "sem dados do inversor")

            if cliente.is_connected():
                cliente.publish(TOPIC_DECISAO, decisor.decisao, qos=1, retain=False)

            time.sleep(max(0.0, INTERVALO - (time.monotonic() - inicio)))
    finally:
        # Desconexão limpa não dispara o Last Will, então avisa manualmente
        log.info("Encerrando...")
        if cliente.is_connected():
            cliente.publish(TOPIC_STATUS, "offline", qos=1, retain=True).wait_for_publish(2)
        cliente.loop_stop()
        cliente.disconnect()
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
