#include <esp_now.h>
#include <WiFi.h>

// Tu MAC Address
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0x2B, 0xDE, 0x08};

// Estructura actualizada (Usamos int para los letreros por seguridad de memoria)
typedef struct struct_message {
  float x;
  float y;
  float theta;
  int tipo_terreno;
  int tipo_roca;
  int letrero_fin;
  int letrero_inicio; 
} struct_message;

struct_message telemetria;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  // Comentado para no ensuciar el puerto Serial del Rover
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6); // La MAC siempre es 6
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) return;
}

void loop() {
  if (Serial.available() > 0) {
    String incomingString = Serial.readStringUntil('\n');
    incomingString.trim(); 

    // Extraemos los 7 valores
    int parseados = sscanf(incomingString.c_str(), "%f,%f,%f,%d,%d,%d,%d", 
                           &telemetria.x, 
                           &telemetria.y, 
                           &telemetria.theta, 
                           &telemetria.tipo_terreno, 
                           &telemetria.tipo_roca, 
                           &telemetria.letrero_fin,
                           &telemetria.letrero_inicio);

    if (parseados == 7) {
      esp_now_send(broadcastAddress, (uint8_t *) &telemetria, sizeof(telemetria));
    }
  }
}