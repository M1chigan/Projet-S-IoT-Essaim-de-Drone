import socket
import threading
import time
import argparse
import sys
import os


# -----------------------------------------------------------------------------
# Script: synch_hymne_master.py
# Description: Serveur maître pour synchronisation d'hymne.
# - Accepte les connexions des clients esclaves via TCP.
# - Envoie le message "START" à tous les clients.
# - Attend les "ACK" des clients et exécute `vehicle.hymne()` pour vérifier le délai de communication
# -----------------------------------------------------------------------------

# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
from Drone import Drone


# Ajout de paramètre --f au lancement du code pour définir le nombre de followers attendus
parser = argparse.ArgumentParser(description="angle")
parser.add_argument('--f', type=int, default=1, help='nombre de followers, default: 1')
args = parser.parse_args()
f = args.f


#adresses MAC Bluetooth des clients connus (non utilisées dans cette version TCP)
pcfixe_simeon = "60:45:2E:FC:BF:F1"
pi_simeon = "2C:CF:67:A2:82:5D"
portable_simeon = "4C:03:4F:E4:76:53"
pi_spacex = "2C:CF:67:8D:E2:A0"  
pi_walle = "2C:CF:67:85:CB:51"
pi_ricardo = "2C:CF:67:77:35:81"
pi_futuna = "88:A2:9E:01:29:97"

# Configuration réseau pour TCP sur WiFi
MASTER_IP = "10.42.0.53"  # Adresse du maître (à remplacer si besoin)
SLAVE_IP = "10.42.0.52"   # exemple esclave (pas strictement requis sur le serveur)
PORT = 5005               # Port TCP à utiliser (doit correspondre au client)

# Liste pour suivre tous les clients connectés
clients = []
clients_ack = []
clients_lock = threading.Lock()

def handle_client(client_socket, address):
    """Gère la communication avec un client.
    """
    print(f"Accepted connection from {address}")
    
    # Ajouter le client à la liste
    with clients_lock:
        clients.append(client_socket)
        clients_ack.append(False)
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            
            message = data.decode()
            print(f"Received from {address}: {message}")
            if 'ACK' in message:
                clients_ack[clients.index(client_socket)] = True
            # Renvoie au client
            #client_socket.send(f"Echo: {message}".encode())
            
            # Optionnellement, diffuse à tous les autres clients
            #broadcast_message(f"Broadcast from {address}: {message}", exclude=None)
            
    except Exception as e:
        print(f"Error with client {address}: {e}")
    finally:
        # Supprime le client de la liste lorsqu'il se déconnecte
        with clients_lock:
            if client_socket in clients:
                index = clients.index(client_socket)
                clients.remove(client_socket)
                clients_ack.pop(index)
        client_socket.close()
        print(f"Client {address} disconnected")

def broadcast_message(message, exclude=None):
    """Envoie un message à tous les clients connectés, sauf `exclude`.
    """
    with clients_lock:
        for client in clients[:]:  # Copie de la liste pour éviter la modification pendant l'itération
            if client != exclude:
                try:
                    client.send(message.encode("utf-8"))
                except:
                    # Supprime le client si l'envoi échoue (client probablement déconnecté)
                    index = clients.index(client)
                    clients.remove(client)
                    clients_ack.pop(index)


def wait_for_ack():
    # Attendre que tous les clients aient mis leur flag ack. Ajout d'un timeout.
    start = time.time()
    timeout = 15.0  # secondes
    while True:
        with clients_lock:
            all_ack = len(clients_ack) > 0 and all(clients_ack)
        if all_ack:
            with clients_lock:
                for i in range(len(clients_ack)):
                    clients_ack[i] = False
            print("All clients acknowledged START")
            return True
        if time.time() - start > timeout:
            print(f"wait_for_ack: timeout after {timeout} seconds; acks={clients_ack}")
            return False
        time.sleep(0.1)

def drone_list():
    vehicle = Drone()
    # On attend que le nombre de clients requis soit connecté
    while True:
        with clients_lock:
            connected = len(clients)
        if connected >= f:
            break
        print(f"Waiting for {f - connected} more clients to connect...")
        time.sleep(2)

    time.sleep(2)  # Donne un peu de temps à tous les clients pour être prêts
    print("Diffusion de START aux clients...")
    broadcast_message("START")
    ok = wait_for_ack()
    if not ok:
        print("Tous les clients n'ont pas accusé réception de START dans le délai imparti")
    else:
        # Si tous les clients ont accusé réception, lance l'hymne
        vehicle.hymne()
    



# XCréation du socket serveur TCP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((MASTER_IP, PORT))
server_socket.listen(5)  # Allow up to 5 pending connections

print(f"Server listening on {MASTER_IP}:{PORT} (TCP)...")
print("Waiting for connections...")
drone_listener = threading.Thread(target=drone_list)
drone_listener.daemon = True
drone_listener.start()
try:
    while True:
        # Accepte les nouvelles connexions
        client_socket, address = server_socket.accept()
        print("accepted connection from ", address)
        # Crée un nouveau thread pour chaque client
        client_thread = threading.Thread(target=handle_client, args=(client_socket, address))
        client_thread.daemon = True  # Le thread se fermera lorsque le programme principal se terminera
        client_thread.start()
        
except KeyboardInterrupt:
    print("\nServer shutting down...")
finally:
    server_socket.close()