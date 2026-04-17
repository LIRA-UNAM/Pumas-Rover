#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  float x;
  float y;
  float theta;
  int tipo_terreno;
  int tipo_roca;
  int letrero_fin;
  int letrero_inicio;
} struct_message;

struct_message datosRecibidos;

// Función que se ejecuta cuando llegan datos
// Función que se ejecuta cuando llegan datos
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
  
  // Imprimimos en formato CSV para que el nodo de ROS2 lo lea fácilmente
  Serial.print(datosRecibidos.x);
  Serial.print(",");
  Serial.print(datosRecibidos.y);
  Serial.print(",");
  Serial.print(datosRecibidos.theta);
  Serial.print(",");
  Serial.print(datosRecibidos.tipo_terreno);
  Serial.print(",");
  Serial.print(datosRecibidos.tipo_roca);
  Serial.print(",");
  Serial.println(datosRecibidos.letrero_fin);
  Serial.print(",");
  Serial.println(datosRecibidos.letrero_inicio);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // El ESP32 se queda escuchando en segundo plano
}