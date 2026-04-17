#include <WiFi.h>
#include <esp_mac.h> // Esta librería es obligatoria en Core 3.x para leer la MAC

void setup(){
  Serial.begin(115200);
  delay(1000); // Le damos 1 segundo al chip para despertar
  
  uint8_t mac[6];
  // Leemos la MAC grabada de fábrica para el modo Estación Wi-Fi
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  
  Serial.println(" ");
  Serial.print("La Dirección MAC de esta base es: ");
  // Formateamos los números en Hexadecimal separados por dos puntos
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop(){}