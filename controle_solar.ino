/*
 * controle_solar.ino
 * ESP32 recebe pelo MQTT a decisão calculada pelo Raspberry Pi e aciona os relés.
 *
 *   "INSUFICIENTE" -> RELÉ 1 ligado, RELÉ 2 desligado
 *   "SUFICIENTE"   -> RELÉ 2 ligado, RELÉ 1 desligado
 *
 * Regras de segurança (ficam aqui, não dependem da rede nem do Pi):
 *   - Os dois relés NUNCA ficam ligados ao mesmo tempo: sempre desliga os dois,
 *     espera o tempo morto e só então liga o novo.
 *   - O relé 2 só pode ser ligado TEMPO_MIN_PARA_RELE2 após a última troca (protege os compressores).
 *     A volta para o relé 1 é sempre imediata.
 *   - Sem decisão do Pi por TEMPO_SEM_DECISAO, Pi "offline", ou sem WiFi/MQTT -> relé 1.
 *   - Só aceita "SUFICIENTE" ou "INSUFICIENTE"; qualquer outra mensagem é ignorada.
 *   - Ao ligar, os dois relés começam desligados e o primeiro modo é sempre o relé 1.
 *
 * Biblioteca: PubSubClient (Nick O'Leary) + pacote esp32 da Espressif.
 * Placa: "ESP32 Dev Module".
 */

#include <WiFi.h>
#include <PubSubClient.h>

// ---------- Rede local ----------
const char* WIFI_SSID     = "SEU_WIFI";
const char* WIFI_PASSWORD = "SUA_SENHA";

// ---------- Broker MQTT (Mosquitto no Raspberry Pi) ----------
const char* MQTT_BROKER   = "192.168.1.50";  // IP fixo do Raspberry Pi
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "solar";
const char* MQTT_PASSWORD = "troque_esta_senha";

// ---------- Tópicos ----------
const char* TOPIC_DECISAO    = "controle_solar/decisao";           // recebe do Pi
const char* TOPIC_PI_STATUS  = "controle_solar/raspberry/status";  // recebe do Pi: online/offline
const char* TOPIC_RELES      = "controle_solar/reles";             // publica o estado dos relés
const char* TOPIC_ESP_STATUS = "controle_solar/esp32/status";      // online/offline (LWT)

// ---------- Pinos ----------
const int PIN_RELE1 = 26;  // energia insuficiente
const int PIN_RELE2 = 27;  // energia suficiente
// Muitos módulos de relé acionam com LOW. Se o seu for assim, troque para LOW.
const uint8_t RELE_LIGADO = HIGH;

// ---------- Temporização ----------
const unsigned long TEMPO_MIN_PARA_RELE2  = 180000;  // 3 min desde a última troca
const unsigned long TEMPO_MORTO_RELES     = 2000;    // ms com os dois desligados na troca
const unsigned long TEMPO_SEM_DECISAO     = 30000;   // sem mensagem do Pi -> relé 1
const unsigned long INTERVALO_RECONEXAO   = 5000;    // tentativa de reconectar ao MQTT
const unsigned long INTERVALO_WIFI        = 30000;   // tentativa de reconectar ao WiFi
const unsigned long INTERVALO_PUBLICACAO  = 10000;   // publica o estado dos relés

WiFiClient espClient;
PubSubClient mqtt(espClient);

enum Modo { AGUARDANDO, INSUFICIENTE, SUFICIENTE };
Modo modo   = AGUARDANDO;  // estado real dos relés
Modo pedido = AGUARDANDO;  // o que o Pi pediu

unsigned long ultimaTroca = 0;
unsigned long ultimaDecisao = 0;
unsigned long ultimaTentativaMqtt = 0;
unsigned long ultimaTentativaWifi = 0;
unsigned long ultimaPublicacao = 0;
bool wifiConectado = false;

const char* nomeModo(Modo m) {
  switch (m) {
    case INSUFICIENTE: return "INSUFICIENTE";
    case SUFICIENTE:   return "SUFICIENTE";
    default:           return "AGUARDANDO";
  }
}

// ================= Relés =================

void desligarReles() {
  digitalWrite(PIN_RELE1, !RELE_LIGADO);
  digitalWrite(PIN_RELE2, !RELE_LIGADO);
}

void publicarReles() {
  if (!mqtt.connected()) return;
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"modo\":\"%s\",\"pedido\":\"%s\",\"rele1\":\"%s\",\"rele2\":\"%s\",\"rssi\":%d}",
           nomeModo(modo), nomeModo(pedido),
           modo == INSUFICIENTE ? "ON" : "OFF", modo == SUFICIENTE ? "ON" : "OFF",
           WiFi.RSSI());
  mqtt.publish(TOPIC_RELES, payload);
}

// Única função que liga relés: sempre desliga os dois antes de ligar o novo
void mudarModo(Modo novo) {
  if (novo == modo) return;

  desligarReles();
  if (modo != AGUARDANDO) delay(TEMPO_MORTO_RELES);

  if (novo == INSUFICIENTE)    digitalWrite(PIN_RELE1, RELE_LIGADO);
  else if (novo == SUFICIENTE) digitalWrite(PIN_RELE2, RELE_LIGADO);

  Serial.printf(">>> Troca: %s -> %s\n", nomeModo(modo), nomeModo(novo));
  modo = novo;
  ultimaTroca = millis();
  publicarReles();
}

void aplicarPedido() {
  if (pedido == AGUARDANDO) return;

  // Partida sempre pelo relé 1
  if (modo == AGUARDANDO) {
    mudarModo(INSUFICIENTE);
    return;
  }

  if (pedido == INSUFICIENTE) {
    mudarModo(INSUFICIENTE);  // volta imediata
  } else if (modo != SUFICIENTE && millis() - ultimaTroca >= TEMPO_MIN_PARA_RELE2) {
    mudarModo(SUFICIENTE);
  }
}

// Volta para o relé 1 por segurança (registra o motivo uma vez)
void forcarInsuficiente(const char* motivo) {
  if (pedido != INSUFICIENTE) {
    Serial.printf("Segurança: %s -> relé 1\n", motivo);
    pedido = INSUFICIENTE;
  }
}

// ================= MQTT =================

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (strcmp(topic, TOPIC_DECISAO) == 0) {
    if (msg == "SUFICIENTE") {
      pedido = SUFICIENTE;
    } else if (msg == "INSUFICIENTE") {
      pedido = INSUFICIENTE;
    } else {
      Serial.printf("Decisão desconhecida ignorada: '%s'\n", msg.c_str());
      return;
    }
    ultimaDecisao = millis();
  } else if (strcmp(topic, TOPIC_PI_STATUS) == 0) {
    Serial.printf("Raspberry Pi: %s\n", msg.c_str());
    if (msg == "offline") forcarInsuficiente("Raspberry Pi offline");
  }
}

void conectarMqtt() {
  Serial.print("Conectando ao broker MQTT... ");
  // Last Will: o broker publica "offline" se o ESP32 cair
  if (mqtt.connect("esp32-reles", MQTT_USER, MQTT_PASSWORD,
                   TOPIC_ESP_STATUS, 1, true, "offline")) {
    Serial.println("conectado!");
    mqtt.publish(TOPIC_ESP_STATUS, "online", true);
    mqtt.subscribe(TOPIC_DECISAO, 1);
    mqtt.subscribe(TOPIC_PI_STATUS, 1);
    publicarReles();
  } else {
    Serial.printf("falhou, rc=%d\n", mqtt.state());
  }
}

// Mantém WiFi e MQTT sem bloquear o loop
void manterConexoes() {
  unsigned long agora = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConectado) {
      Serial.println("WiFi desconectado.");
      wifiConectado = false;
    }
    if (agora - ultimaTentativaWifi >= INTERVALO_WIFI) {
      ultimaTentativaWifi = agora;
      WiFi.reconnect();
    }
    return;
  }

  if (!wifiConectado) {
    wifiConectado = true;
    Serial.printf("WiFi conectado. IP: %s\n", WiFi.localIP().toString().c_str());
  }

  if (mqtt.connected()) {
    mqtt.loop();
  } else if (agora - ultimaTentativaMqtt >= INTERVALO_RECONEXAO) {
    ultimaTentativaMqtt = agora;
    conectarMqtt();
  }
}

// ================= Setup / Loop =================

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELE1, OUTPUT);
  pinMode(PIN_RELE2, OUTPUT);
  desligarReles();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(callback);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(5);

  Serial.println("Controle de relés iniciado. Aguardando decisão do Raspberry Pi...");
}

void loop() {
  manterConexoes();

  // Watchdog: sem decisão recente do Pi (inclui queda de WiFi/MQTT) -> relé 1
  if (millis() - ultimaDecisao > TEMPO_SEM_DECISAO) {
    forcarInsuficiente("sem decisão do Raspberry Pi");
  }

  aplicarPedido();

  if (millis() - ultimaPublicacao >= INTERVALO_PUBLICACAO) {
    ultimaPublicacao = millis();
    publicarReles();
  }
}
