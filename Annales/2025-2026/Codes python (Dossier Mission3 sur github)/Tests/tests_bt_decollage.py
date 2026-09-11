import argparse
import socket

import sys
import os
import time
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone
vehicle = Drone()

# -----------------------------------------------------------------------------
# Script: tests_bt_decollage.py
# Description: Test de décollage synchronisé.
# - Synchronise le décollage et l'atterrissage avec un suiveur via Bluetooth.
# - Les variables MAC servent à identifier les appareils de test.
# -----------------------------------------------------------------------------

# Pas utilisé
parser = argparse.ArgumentParser(description="angle")
parser.add_argument('--angle', type=int, default=180, help='Angle de suivi, default: 180 ou SUD')
parser.add_argument('--dist', type=int, default=180, help='Distance de suivi, default: 10m')
args = parser.parse_args()
angle = args.angle
dist = args.dist

pcfixe_simeon = "60:45:2E:FC:BF:F1"
pi_simeon = "2C:CF:67:A2:82:5D"
portable_simeon = "4C:03:4F:E4:76:53"
pi_spacex = "DC:A6:32:49:79:E3"
pi_walle = "2C:CF:67:85:CB:51"

pi_ricardo = "2C:CF:67:77:35:81"

#Connection Bluetooth au maitre
client_socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
client_socket.connect((pi_ricardo, 5))

# Attente du message START du maitre
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            exit(0)
        data = data.decode("utf-8")
        if "START" in data:
            client_socket.send("ACK".encode("utf-8"))
            print("Received START, proceeding with takeoff")
            break
except Exception as e:
    print("Error:", e)
    exit(0)



#Décollage
vehicle.request_global_position_stream(rate_hz=10)

vehicle.set_mode("GUIDED")
vehicle.arm_and_takeoff(10)
vehicle.interrupt()
vehicle.set_yaw(0)

# Attente du message HIGH du maitre
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            vehicle.set_mode("LAND")
            exit(0)
        data = data.decode("utf-8")
        if "HIGH" in data:
            print("Received HIGH command, ascending")
            vehicle.set_mode("LAND")#Atterrissage
            client_socket.send("ACK".encode("utf-8"))
            break
except Exception as e:
    vehicle.set_mode("LAND")
    print("Error:", e)
    exit(0)