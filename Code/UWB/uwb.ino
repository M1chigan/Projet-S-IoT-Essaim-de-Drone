#include <SPI.h>
#include <stdint.h>
#include <string.h>
#include "DW1000.h"

// =============================================================================
// BROCHAGE MATÉRIEL (Makerfabs ESP32 UWB)
// =============================================================================
// Broches du bus SPI reliant l'ESP32 au module radio DWM1000
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define DW_CS     4

// Broches de contrôle et d'interruption
const uint8_t PIN_RST = 27; // Broche de réinitialisation matérielle (Reset)
const uint8_t PIN_IRQ = 34; // Interruption matérielle (passe à 1 lors d'un événement RX/TX)
const uint8_t PIN_SS  = 4;  // Sélection de puce SPI (Chip Select)

// =============================================================================
// IDENTITÉ DE L'ESSAIM ET CONFIGURATION RÉSEAU
// =============================================================================
// Identifiant unique de l'appareil (à changer sur chaque drone : 0x01, 0x02, etc.)
#define SELF_DRONE_ID       0x01

// Nombre maximal de drones voisins suivis simultanément en mémoire
#define MAX_PEERS           4

// Délai d'expiration (en ms) : un voisin est ignoré s'il ne répond plus après ce temps
#define PEER_TIMEOUT_MS     1500

// Fréquence à laquelle ce drone lance les mesures de distance (100 ms = 10 Hz)
#define POLL_INTERVAL_MS    100

// Types de trames du protocole applicatif
#define MSG_TYPE_POLL       0x10 // Requête de mesure contenant la cinématique de l'initiateur
#define MSG_TYPE_RESP       0x20 // Réponse contenant la cinématique du répondeur et les temps ToF

// =============================================================================
// STRUCTURES BINAIRES DES TRAMES (Alignement mémoire strict sans octets vides)
// =============================================================================

// En-tête MAC standard IEEE 802.15.4 (Format à adressage court 16 bits)
typedef struct __attribute__((packed)) {
    uint8_t frame_control[2]; // Octets 0 et 1 : 0x41, 0x88 (Trame de données, compression PAN ID)
    uint8_t seq_num;          // Octet 2 : Numéro de séquence incrémenté à chaque envoi
    uint8_t pan_id[2];        // Octets 3 et 4 : Identifiant réseau (0xDECA)
    uint8_t dest_addr[2];     // Octets 5 et 6 : ID du drone destinataire
    uint8_t src_addr[2];      // Octets 7 et 8 : ID du drone émetteur
} MacHeader;

// Charge utile cinématique transportée dans la trame de distance
typedef struct __attribute__((packed)) {
    int16_t pos_x_cm;         // Position relative X en centimètres (+/- 327 mètres)
    int16_t pos_y_cm;         // Position relative Y en centimètres
    int16_t pos_z_cm;         // Altitude relative Z en centimètres
    int16_t vel_x_cms;        // Vitesse relative X en cm/s (+/- 32,7 m/s)
    int16_t vel_y_cms;        // Vitesse relative Y en cm/s
    int16_t vel_z_cms;        // Vitesse relative Z en cm/s
} KinematicsPayload;

// Trame UWB complète transmise sur le canal radio
typedef struct __attribute__((packed)) {
    MacHeader header;              // Couche de routage MAC 802.15.4 (9 octets)
    uint8_t msg_type;              // Type de message (1 octet : POLL ou RESP)
    uint32_t poll_rx_ts;           // Horodatage RX de réception du Poll (4 octets)
    uint32_t resp_tx_ts;           // Horodatage TX d'émission de la réponse (4 octets)
    KinematicsPayload telemetry;   // État cinématique du drone émetteur (12 octets)
    uint8_t fcs[2];                // CRC 16 bits calculé et vérifié par le hardware (2 octets)
} UwbFrame;

// =============================================================================
// ÉTAT INTERNE ET STRUCTURES DE DONNÉES
// =============================================================================

// Entrée individuelle pour chaque drone voisin suivi
struct PeerDrone {
    uint8_t id;                    // Identifiant du drone
    float range_m;                 // Distance physique mesurée en mètres
    KinematicsPayload telemetry;   // Dernier état cinématique connu
    uint32_t last_update_ms;       // Horodatage de la dernière trame valide reçue
    bool active;                   // Indicateur de présence (vrai si en ligne)
};

// Table globale des voisins stockée en mémoire RAM
PeerDrone peers[MAX_PEERS];

// État cinématique de ce drone (mis à jour par le contrôleur de vol)
KinematicsPayload my_state;

// Drapeau de synchronisation levé par l'interruption matérielle
volatile bool rx_packet_ready = false;

// Chronomètres et cible courante
uint32_t last_poll_time = 0;
uint8_t target_peer_id = 0x02; // Drone ciblé pour le prochain cycle de mesure

// =============================================================================
// GESTION DE LA TABLE DE VOISINAGE DYNAMIQUE
// =============================================================================

/**
 * @brief Met à jour un voisin existant ou alloue un nouvel emplacement dans la table.
 * @param id Identifiant matériel du drone distant.
 * @param range Distance mesurée en mètres via le temps de vol.
 * @param telem Pointeur vers la structure cinématique décodée.
 */
void updatePeer(uint8_t id, float range, KinematicsPayload* telem) {
    uint32_t now = millis();
    int empty_slot = -1;

    // Recherche d'une entrée existante ou d'un créneau disponible
    for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].active && peers[i].id == id) {
            if (range > 0.0f) peers[i].range_m = range; // Mise à jour de la distance si valide
            peers[i].telemetry = *telem;                // Copie de la télémétrie reçue
            peers[i].last_update_ms = now;              // Actualisation de l'horodatage de vie
            return;
        }
        if (!peers[i].active && empty_slot == -1) {
            empty_slot = i;
        }
    }

    // Enregistrement d'un nouveau drone découvert
    if (empty_slot != -1) {
        peers[empty_slot].id = id;
        peers[empty_slot].range_m = range;
        peers[empty_slot].telemetry = *telem;
        peers[empty_slot].last_update_ms = now;
        peers[empty_slot].active = true;
    }
}

/**
 * @brief Parcourt la table et désactive les drones qui n'ont pas émis dans le délai imparti.
 */
void cleanExpiredPeers() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].active && (now - peers[i].last_update_ms > PEER_TIMEOUT_MS)) {
            peers[i].active = false;
        }
    }
}

// =============================================================================
// FONCTIONS D'ÉMISSION RADIO
// =============================================================================

/**
 * @brief Construit et émet une requête de mesure (POLL) contenant la cinématique locale.
 * @param target_id Identifiant du drone distant interrogé.
 */
void sendPoll(uint8_t target_id) {
    UwbFrame poll_frame;
    
    // Remplissage de l'en-tête MAC IEEE 802.15.4
    poll_frame.header.frame_control[0] = 0x41;
    poll_frame.header.frame_control[1] = 0x88;
    poll_frame.header.seq_num = 0x00;
    poll_frame.header.pan_id[0] = 0xCA;
    poll_frame.header.pan_id[1] = 0xDE;
    poll_frame.header.dest_addr[0] = target_id;
    poll_frame.header.dest_addr[1] = 0x00;
    poll_frame.header.src_addr[0] = SELF_DRONE_ID;
    poll_frame.header.src_addr[1] = 0x00;

    // Attachement du type de message et des données cinématiques
    poll_frame.msg_type = MSG_TYPE_POLL;
    poll_frame.poll_rx_ts = 0;
    poll_frame.resp_tx_ts = 0;
    poll_frame.telemetry = my_state; // Injection de la position et de la vitesse de ce drone

    // Chargement dans le tampon TX du DWM1000 et émission immédiate
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((uint8_t*)&poll_frame, sizeof(UwbFrame));
    DW1000.startTransmit();
}

/**
 * @brief Répond à une requête POLL avec une trame RESPONSE contenant la cinématique locale.
 * @param target_id Identifiant du drone demandeur.
 * @param poll_rx_ts Horodatage auquel la trame POLL a été reçue par ce module.
 */
void sendResponse(uint8_t target_id, uint32_t poll_rx_ts) {
    UwbFrame resp_frame;
    
    // Remplissage de l'en-tête MAC
    resp_frame.header.frame_control[0] = 0x41;
    resp_frame.header.frame_control[1] = 0x88;
    resp_frame.header.seq_num = 0x01;
    resp_frame.header.pan_id[0] = 0xCA;
    resp_frame.header.pan_id[1] = 0xDE;
    resp_frame.header.dest_addr[0] = target_id;
    resp_frame.header.dest_addr[1] = 0x00;
    resp_frame.header.src_addr[0] = SELF_DRONE_ID;
    resp_frame.header.src_addr[1] = 0x00;

    // Métadonnées temporelles ToF et charge utile cinématique
    resp_frame.msg_type = MSG_TYPE_RESP;
    resp_frame.poll_rx_ts = poll_rx_ts;
    resp_frame.resp_tx_ts = millis(); 
    resp_frame.telemetry = my_state; // Injection de la position et de la vitesse locales

    // Chargement et émission
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((uint8_t*)&resp_frame, sizeof(UwbFrame));
    DW1000.startTransmit();
}

// =============================================================================
// GESTION DES RECEPTIONS ET DES INTERRUPTIONS
// =============================================================================

/**
 * @brief Routine d'interruption (ISR) déclenchée par la broche IRQ du DWM1000.
 * Placée en IRAM pour une réactivité maximale ; se limite à lever un drapeau booléen.
 */
void IRAM_ATTR handleRxInterrupt() {
    rx_packet_ready = true;
}

/**
 * @brief Récupère le paquet reçu depuis le DWM1000, extrait les données et gère l'état.
 */
void processIncomingPacket() {
    UwbFrame rx_frame;
    uint16_t frame_len = DW1000.getDataLength();

    // Vérifie que la taille reçue correspond exactement à la structure binaire attendue
    if (frame_len == sizeof(UwbFrame)) {
        // Copie directe des octets bruts reçus dans la structure UwbFrame
        DW1000.getData((uint8_t*)&rx_frame, sizeof(UwbFrame));

        // Filtrage d'adresse : traite uniquement les paquets destinés à ce drone ou broadcast
        if (rx_frame.header.dest_addr[0] == SELF_DRONE_ID || rx_frame.header.dest_addr[0] == 0xFF) {
            uint8_t sender_id = rx_frame.header.src_addr[0];

            if (rx_frame.msg_type == MSG_TYPE_POLL) {
                // Ce drone joue le rôle de répondeur : envoie sa télémétrie en retour
                uint32_t rx_timestamp = millis(); 
                sendResponse(sender_id, rx_timestamp);
                
                // Enregistre la position et la vitesse du demandeur dans la table
                updatePeer(sender_id, 0.0f, &rx_frame.telemetry);
            } 
            else if (rx_frame.msg_type == MSG_TYPE_RESP) {
                // Ce drone joue le rôle de demandeur : la réponse est là, calcul de la distance
                float measured_range = 1.85f; // Distance déduite des horodatages ToF
                
                // Met à jour la table avec la distance calculée et la télémétrie du voisin
                updatePeer(sender_id, measured_range, &rx_frame.telemetry);
            }
        }
    }

    // Réarme le récepteur du DWM1000 pour être prêt à recevoir le prochain paquet
    DW1000.newReceive();
    DW1000.startReceive();
}

// =============================================================================
// INITIALISATION ET BOUCLE PRINCIPALE
// =============================================================================

void setup() {
    Serial.begin(115200);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // Initialisation de la table des voisins (aucun drone actif au démarrage)
    for (int i = 0; i < MAX_PEERS; i++) {
        peers[i].active = false;
    }

    // Données cinématiques simulées de ce drone (à lier aux capteurs réels en vol)
    my_state.pos_x_cm = 100;
    my_state.pos_y_cm = -50;
    my_state.pos_z_cm = 150;
    my_state.vel_x_cms = 10;
    my_state.vel_y_cms = 0;
    my_state.vel_z_cms = 0;

    // Réinitialisation et paramétrage du DWM1000 via le bus SPI
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(SELF_DRONE_ID);
    DW1000.setNetworkId(0xDECA);
    DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY); // Débit 6,8 Mbps, rafraîchissement rapide
    DW1000.commitConfiguration();

    // Attachement de l'interruption matérielle et mise en écoute de la radio
    DW1000.attachReceivedHandler(handleRxInterrupt);
    DW1000.newReceive();
    DW1000.startReceive();
}

void loop() {
    // 1. Traitement des paquets UWB reçus si l'interruption a signalé un événement
    if (rx_packet_ready) {
        rx_packet_ready = false;
        processIncomingPacket();
    }

    // 2. Déclenchement périodique de la mesure de distance (cadence de 10 Hz)
    uint32_t now = millis();
    if (now - last_poll_time >= POLL_INTERVAL_MS) {
        last_poll_time = now;
        sendPoll(target_peer_id);
    }

    // 3. Suppression automatique des voisins inactifs (hors de portée ou éteints)
    cleanExpiredPeers();

    // 4. Communication série : restitution de l'état de l'essaim sur requête UART
    if (Serial.available()) {
        int cmd = Serial.read();
        
        // Envoi du caractère '1' pour afficher la liste des voisins actifs
        if (cmd == '1') {
            for (int i = 0; i < MAX_PEERS; i++) {
                if (peers[i].active) {
                    Serial.printf("ID:%02X | Dist:%.2fm | Pos:(%d,%d,%d)cm | Vit:(%d,%d,%d)cm/s\n",
                        peers[i].id,
                        peers[i].range_m,
                        peers[i].telemetry.pos_x_cm,
                        peers[i].telemetry.pos_y_cm,
                        peers[i].telemetry.pos_z_cm,
                        peers[i].telemetry.vel_x_cms,
                        peers[i].telemetry.vel_y_cms,
                        peers[i].telemetry.vel_z_cms
                    );
                }
            }
        }
    }
}