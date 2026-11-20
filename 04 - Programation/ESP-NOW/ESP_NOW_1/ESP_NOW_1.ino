#include <esp_now.h>
#include <WiFi.h>

// Remplacez cette adresse par l'adresse MAC de l'autre ESP32
uint8_t peer1_mac[] = {0xCC, 0xDB, 0xA7, 0x07, 0xB9, 0x78}; 

// Structure des données échangées
typedef struct struct_message {
    int id;
    int counter;
    char message[32];
} struct_message;

struct_message myData;
struct_message incomingData;

// Callback appelé lorsque des données sont envoyées
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Statut d'envoi : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Succès" : "Échec");
}

// Callback appelé lorsque des données sont reçues
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
  Serial.println("--- Message reçu ---");
  Serial.print("ID du pair : "); Serial.println(incomingData.id);
  Serial.print("Compteur : "); Serial.println(incomingData.counter);
  Serial.print("Message : "); Serial.println(incomingData.message);
  Serial.println("--------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialisation propre du Wi-Fi en mode Station
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);

  // Initialiser ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur d'initialisation ESP-NOW");
    return;
  }

  // Enregistrement des callbacks avec conversion explicite pour le Core ESP32 v3.x
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Ajouter le pair (l'autre ESP32)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, peer1_mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Échec de l'ajout du pair");
    return;
  }
}

void loop() {
  myData.id = 2; // Mettez 1 sur la première carte, 2 sur la seconde
  myData.counter++;
  snprintf(myData.message, sizeof(myData.message), "Je m'appelle ESP32 !");

  esp_err_t result = esp_now_send(peer1_mac, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Message envoyé avec succès");
  } else {
    Serial.println("Erreur lors de l'envoi");
  }

  delay(2000);
}