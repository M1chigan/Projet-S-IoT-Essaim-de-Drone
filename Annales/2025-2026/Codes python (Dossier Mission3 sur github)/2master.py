import socket
import threading
import time
import argparse

import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from pymavlink import mavutil
from Drone import Drone

parser = argparse.ArgumentParser(description="angle")
parser.add_argument('--f', type=int, default=1, help='nombre de followers, default: 1')
args = parser.parse_args()
f = args.f

# Variable pour mode test, pas de décollage ni de contrôle de mouvement
test = True
if test:
    print("TEST")


#adresses MAC Bluetooth des clients connus (non utilisées dans cette version TCP)
#pcfixe_simeon = "60:45:2E:FC:BF:F1"
#pi_simeon = "2C:CF:67:A2:82:5D"
#portable_simeon = "4C:03:4F:E4:76:53"
#pi_spacex = "DC:A6:32:49:79:E3"
#pi_walle = "2C:CF:67:85:CB:51"
#pi_ricardo = "2C:CF:67:77:35:81"

# List to keep track of all connected clients
clients = []
clients_ack = []
clients_lock = threading.Lock()
d_C = 5
rec_dist = False


# Fonction de gestion d'un client connecté
def handle_client(client_socket, address):
    # Accusé d'acceptation d'une connexion client
    print(f"Accepted connection from {address}")
    
    # Add client to the list
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
            if message.startswith("D:"):
                try:
                    global d_C, rec_dist
                    d_C = float(message.split(':',1)[1])
                    rec_dist = True
                except:
                    pass
                
            
            # Optionally, broadcast to all other clients
            broadcast_message(f"Broadcast from {address}: {message}", exclude=None)
            
    except Exception as e:
        print(f"Error with client {address}: {e}")
    finally:
        # Remove client from list when disconnected
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
        for client in clients[:]:  # Copy list to avoid modification during iteration
            if client != exclude:
                try:
                    client.send(message.encode("utf-8"))
                except:
                    # Remove client if sending fails (client probably disconnected)
                    index = clients.index(client)
                    clients.remove(client)
                    clients_ack.pop(index)
        print(f"Broadcasted: {message}")


def wait_for_ack(timeout=15.0):
    # Attendre que tous les clients mettent leur flag ack. Ajout d'un timeout.
    start = time.time()
    while True:
        with clients_lock:
            all_ack = len(clients_ack) > 0 and all(clients_ack)
        if all_ack:
            with clients_lock:
                for i in range(len(clients_ack)):
                    clients_ack[i] = False
            return True
        if time.time() - start > timeout:
            print(f"wait_for_ack: timeout after {timeout} seconds; acks={clients_ack}")
            return False
        time.sleep(0.1)  # Avoid busy waiting


# Pas utilisé dans cette version
def stop_procedure():
    broadcast_message("STOP")


# Thread principal pour la gestion du drone master
def drone_list():
    #Initialisation du drone
    try:
        vehicle = Drone(path = "/dev/ttyACM1")
    except:
        print("default")
        vehicle = Drone()
    
    # Attente des followers (ici toujours 1)
    while len(clients) < f:
        print(f"Waiting for {f - len(clients)} more clients to connect...")
        time.sleep(1)
    
    time.sleep(2)  # Give some time for all clients to be ready
    
    if(not test):
        # Attente mode STABILIZE
        while vehicle.get_mode() != "STABILIZE":
            vehicle.log({"event": "En attente du mode STABILIZE"})

        # Attente mode AUTO
        while vehicle.get_mode() != "AUTO":
            vehicle.log({"event": "En attente du mode AUTO"})
        vehicle.set_mode("GUIDED")
    print("INIT")

    # Envoi de la commande START aux followers
    broadcast_message("START")
    if(test):
        vehicle.request_global_position_stream(rate_hz=1)
    else:
        vehicle.request_global_position_stream(rate_hz=10)
    wait_for_ack() #attente du retour des followers

    # Décollage
    if not test:
        vehicle.arm_and_takeoff(5)
        vehicle.set_yaw(0)
    

    # Envoi de la commande HIGH aux followers
    print('HIGH')
    broadcast_message("HIGH")
    wait_for_ack() #attente du retour des followers
    print("All followers are ready")
    vehicle.bip_melodie("T70L1G6")

    # Boucle principale d'envoi des données aux followers
    while True:
        try:
            lat,lon,alt,vx,vy,vz = vehicle.get_pose_vel(to=0.2)
            lat = lat/1e7
            lon = lon/1e7
            alt = vehicle.get_alt_lidar()
            d_C = 0
            data_str = f"{alt}"#Dans cette version on n'envoie que l'altitude
            
            broadcast_message(data_str)
        except Exception as e:
            print(f"Error in drone_list: {e}")
            time.sleep(1)




# Network configuration for TCP over WiFi (same defaults as synch_hymne_master)
MASTER_IP = "10.42.0.55"
PORT = 5005

# Create TCP server socket (IPv4)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(("0.0.0.0", PORT))
server_socket.listen(5)  # Allow up to 5 pending connections
print(f"Server listening on {MASTER_IP}:{PORT} (TCP)...")
print("Waiting for connections...")

#Démarrage du thread de gestion du drone master
drone_listener = threading.Thread(target=drone_list)
drone_listener.daemon = True
drone_listener.start()
try:
    while True:
        # Accept new connections
        client_socket, address = server_socket.accept()
        
        # Create a new thread for each client
        client_thread = threading.Thread(target=handle_client, args=(client_socket, address))
        client_thread.daemon = True  # Thread will close when main program exits
        client_thread.start()
        
except KeyboardInterrupt:
    print("\nServer shutting down...")
finally:
    server_socket.close()