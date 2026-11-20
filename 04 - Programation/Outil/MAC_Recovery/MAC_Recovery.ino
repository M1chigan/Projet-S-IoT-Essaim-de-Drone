#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000); // Laisse le temps au port série et à l'alimentation de se stabiliser

  // Réinitialisation propre du Wi-Fi
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print("Adresse MAC : ");
  Serial.println(WiFi.macAddress());
}

void loop() {}