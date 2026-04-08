#include <esp_now.h>
#include <WiFi.h>

// ¡REEMPLAZA ESTO CON LA MAC QUE OBTUVISTE EN EL PASO 2!
uint8_t broadcastAddress[] = {0xC0, 0xCD, 0xD6, 0x8D, 0x50, 0x24};
// Estructura de datos (Debe ser idéntica en emisor y receptor)
typedef struct struct_message {
  float x;
  float y;
  float theta;
  int tipo_terreno; // 0=Normal, 1=Valle, 2=Surco, 3=Pendiente
  int tipo_roca;    // 0=Ninguna, 1=Roja, 2=Azul, 3=Verde
  int letrero_fin;  // 0=No, 1=Sí
} struct_message;

struct_message telemetria;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Estado del envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Registrar el receptor
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Error añadiendo el peer");
    return;
  }
}

void loop() {
  // Simulando datos del mapa y sensores
  telemetria.x = 2.50;
  telemetria.y = 1.20;
  telemetria.theta = 90.0;
  telemetria.tipo_terreno = 0;
  telemetria.tipo_roca = 1; // Encontró roca roja
  telemetria.letrero_fin = 0;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &telemetria, sizeof(telemetria));
  
  delay(200); // Enviamos datos a 5Hz (cada 200ms)
}