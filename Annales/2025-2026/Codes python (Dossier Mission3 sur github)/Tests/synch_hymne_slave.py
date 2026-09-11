import argparse
import socket

import sys
import os
import time
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone
try:
    vehicle = Drone(path = "/dev/ttyACM1")
except:
    print("default")
    vehicle = Drone()

# -----------------------------------------------------------------------------
# Script: synch_hymne_slave.py
# Description: Client esclave pour synchronisation d'hymne.
# - Se connecte au serveur maître via TCP et attend le message "START".
# - Répond par "ACK" et exécute `vehicle.hymne()` pour vérifier le délai de communication
# -----------------------------------------------------------------------------


# Obsolète: les paramètres angle et dist ne sont pas utilisés ici
parser = argparse.ArgumentParser(description="angle")
parser.add_argument('--angle', type=int, default=180, help='Angle de suivi, default: 180 ou SUD')
parser.add_argument('--dist', type=int, default=180, help='Distance de suivi, default: 10m')
args = parser.parse_args()
angle = args.angle
dist = args.dist


# Adresses MAC Bluetooth des clients connus (non utilisées dans cette version TCP)
pcfixe_simeon = "60:45:2E:FC:BF:F1"
pi_simeon = "2C:CF:67:A2:82:5D"
portable_simeon = "4C:03:4F:E4:76:53"
pi_spacex = "2C:CF:67:8D:E2:A0" 
pi_walle = "2C:CF:67:85:CB:51"
pi_futuna = "88:A2:9E:01:29:97"
pi_ricardo = "2C:CF:67:77:35:81"

# Configuration réseau pour TCP sur WiFi
MASTER_IP = "10.42.0.53"  # Adresse du maître (à remplacer si besoin)
SLAVE_IP = "10.42.0.52"   # Adresse de l'esclave (pas strictement requis sur le client)
PORT = 5005               # Port TCP à utiliser (doit correspondre au serveur)

def _create_tcp_socket(timeout=20):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    return s

# Essais multiples de connection au serveur maître
client_socket = None
max_attempts = 6
for attempt in range(1, max_attempts + 1):
    client_socket = _create_tcp_socket()
    try:
        print(f"Attempting to connect to {MASTER_IP}:{PORT} (attempt {attempt})...")
        client_socket.connect((MASTER_IP, PORT))
        print(f"Connected to master at {MASTER_IP}:{PORT}")
        break
    except Exception as e:
        print(f"Connection attempt {attempt} failed: {e}")
        try:
            client_socket.close()
        except Exception:
            pass
        if attempt < max_attempts:
            # backoff simple avant nouvelle tentative
            time.sleep(1 + attempt)
            continue
        else:
            print("Unable to connect after retries, exiting")
            sys.exit(1)


# Attente du message "START" du maître
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            print("Master closed connection, exiting")
            exit(0)
        raw = data
        try:
            data = data.decode("utf-8")
        except Exception:
            data = str(data)
        print(f"Slave received: {data}")

        # Message START reçu
        if "START" in data:
            try:
                # Envoie ACK au maître
                client_socket.send("ACK".encode("utf-8"))
                print("Sent ACK to master")
            except Exception as e:
                print(f"Failed to send ACK: {e}")
            # Lance l'hymne puis quitte le code
            print("Received START, proceeding with takeoff")
            vehicle.hymne()
            break
except Exception as e:
    print("Error in receive loop:", e)
    exit(0)