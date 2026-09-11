import socket
import threading
import time
import argparse
import sys
import os
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone

# -----------------------------------------------------------------------------
# Script: test_decollage_master.py
# Description: Serveur maître pour tests de décollage coordonnés.
# - Accepte des connexions Bluetooth (ou socket selon la plateforme) de followers.
# - Coordonne l'envoi de commandes START/HIGH/STOP et attend les ACK des clients.
# - Les connections Bluetooth n'étaient pas fonctionnelles, utilisation de socket TCP recommandée.
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="angle")
parser.add_argument('--f', type=int, default=1, help='nombre de followers, default: 1')
args = parser.parse_args()
f = args.f

#adresses MAC Bluetooth des clients connus (non utilisées dans cette version TCP)
pcfixe_simeon = "60:45:2E:FC:BF:F1"
pi_simeon = "2C:CF:67:A2:82:5D"
portable_simeon = "4C:03:4F:E4:76:53"
pi_spacex = "DC:A6:32:49:79:E3"
pi_walle = "2C:CF:67:85:CB:51"
pi_ricardo = "2C:CF:67:77:35:81"

clients = []
clients_ack = []
clients_lock = threading.Lock()

def handle_client(client_socket, address):
    """Fonction pour gérer la communication avec un client."""
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
            # Echo back to the sender
            #client_socket.send(f"Echo: {message}".encode())
            
            # Optionally, broadcast to all other clients
            #broadcast_message(f"Broadcast from {address}: {message}", exclude=None)
            
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
    """Send message to all connected clients except the excluded one"""
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

# Attend que tous les clients aient envoyé un ACK
def wait_for_ack():
    while True:
        for i in range(len(clients_ack)):
            if not clients_ack[i]:
                break
        else:
            for i in range(len(clients_ack)):
                clients_ack[i] = False
            return  # All clients have acknowledged
        time.sleep(0.1)  # Avoid busy waiting


# Pas utilisé dans cette version
def stop_procedure():
    broadcast_message("STOP")


# Pas utilisé
def avance():
    for i in range(5):
        Drone.move_forward(1.8/5*(i+1))
    for i in range(5):
        Drone.move_forward(1.8-1.8/5*(i+1))
    Drone.set_mode("LOITER")

def drone_list():

    #Initialisation du drone
    vehicle = Drone()
    
    vehicle.request_global_position_stream(rate_hz=10)

    #Attente que le nombre de clients requis soit connecté
    while len(clients) < f:
        print(f"Waiting for {f - len(clients)} more clients to connect...")
        time.sleep(1)
    time.sleep(2)  # Give some time for all clients to be ready

    #Envoi de la commande START aux followers
    broadcast_message("START")
    vehicle.request_global_position_stream(rate_hz=10)
    wait_for_ack() #attente du retour des followers

    #Décollage
    vehicle.set_mode("GUIDED")
    vehicle.arm_and_takeoff(10)
    
    #Interruption si le drone sort du mode GUIDED
    vehicle.interrupt(stop_procedure)
    vehicle.set_yaw(0)
    
    #Envoi de la commande HIGH aux followers
    broadcast_message("HIGH")
    wait_for_ack()#attente du retour des followers


    vehicle.log("All followers acknowledged HIGH command")

    #Atterrissage
    vehicle.set_mode("LAND")



# Create server socket
server_socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_socket.bind((pi_ricardo, 5))  # Bind to any available address
server_socket.listen(5)  # Allow up to 5 pending connections

print("Server listening on channel 5...")
print("Waiting for connections...")
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