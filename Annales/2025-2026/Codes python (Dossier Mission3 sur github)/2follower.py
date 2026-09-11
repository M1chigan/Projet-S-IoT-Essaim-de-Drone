import argparse
import socket

import sys
import os
import time
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
# Import de la classe `Drone` et utilitaires MAVLink
from Drone import Drone
from pymavlink import mavutil
# Instantiate vehicle once
vehicle = Drone()

vehicle.start_uwb_serial()

# -----------------------------------------------------------------------------
# Script: 2follower.py
# Description: Suiveur UWB pour essai en duo (master/follower)
# - Lit deux mesures UWB (d1, d2) et ajuste la vitesse pour suivre le master.
# - Contient une boucle de réception socket TCP pour recevoir données du master.
# -----------------------------------------------------------------------------

# Variable pour passer en mode "test" (sans décollage ni contrôle de vol)
test = True

if test:
    print("TEST")

# Configuration réseau
MASTER_IP = "172.20.10.5"
PORT = 5005


# Contrôles PI pour position et distance
kx_pos = ky_pos = 0.1
ix = iy = iz = 0.05
kz_pos = 0.3
iz_pos = 0.05

kx_d = ky_d = 0.3
ix_d = iy_d = 0.05


d1=0
d2=0
change = False

# Thread de mesure UWB
def measure_thread():
    global d1
    global d2
    global change
    while True:
        d = vehicle.request_uwb_measure()
        #print(f"Measured distance: {d} meters")
        if d is not None:
            msr = d.split(';')
            d1 = float(msr[0].split('=')[1])
            d2 = float(msr[1].split('=')[1])
            change = True
        time.sleep(0.1)








# Creation et connexion du socket client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((MASTER_IP, PORT))

# Lancement du thread de mesure UWB
import threading
threading.Thread(target=measure_thread, daemon=True).start()

# Attente de stabilize et auto avant de passer en guided
if(not test):
    # Attente mode STABILIZE
    while vehicle.get_mode() != "STABILIZE":
        vehicle.log({"event": "En attente du mode STABILIZE"})

    # Attente mode AUTO
    while vehicle.get_mode() != "AUTO":
        vehicle.log({"event": "En attente du mode AUTO"})
    vehicle.set_mode("GUIDED")


# Attente du signal START par le master
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        if "START" in data:
            client_socket.send("ACK".encode("utf-8"))
            break
except Exception as e:
    print("Error:", e)



if test:
    vehicle.request_global_position_stream(rate_hz=1)
else:
    vehicle.request_global_position_stream(rate_hz=10)
    # Décollage
    vehicle.arm_and_takeoff(5)
    vehicle.interrupt()
    vehicle.set_yaw(0)


# Attente du signal HIGH par le master
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        if "HIGH" in data:
            client_socket.send("ACK".encode("utf-8"))
            break
except Exception as e:
    print("Error:", e)


try:
    # Initialisation des variables pour le contrôle
    prev_dx_pos = prev_dy_pos = prev_dz_pos = 0.0
    dx_pos = dy_pos = dz_pos = 0.0
    prev_dx_d = prev_dy_d = 0.0
    dx_d = dy_d = 0.0

    #Boucle principale de réception des données du master et contrôle du drone
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        if "GPS" in data:
            # Server sent a GPS-not-available message; ignore
            continue
        # Si STOP, atterrir et quitter
        elif "STOP" in data:
            vehicle.move_velocity(0, 0, -3, 1)
            vehicle.set_mode("LOITER")
            time.sleep(1)
            exit(0)


        else:
            try:
                #data_str = f"{d_C}:{lat:.6f}:{lon:.6f}:{alt}:{vx:.2f}:{vy:.2f}:{vz:.2f}"
                 master_alt = float(data) # on ne reçoit que l'altitude du master
            except Exception:
                continue
            # Récupération de la position du suiveur
            self_lat, self_lon, self_alt = vehicle.get_pose(to = 0.2)
            self_alt = vehicle.get_alt_lidar()
            vehicle.log({'master_position': f' {master_alt}'})
            vehicle.log({'self_position': f'{self_lat}, {self_lon}, {self_alt}'})
            vehicle.log({"dists":f"{d2}"})

            # Calcul des erreurs de position, pas utilisé dans cette version
            prev_dx_pos, prev_dy_pos, prev_dz_pos = dx_pos, dy_pos, dz_pos
            dz_pos = master_alt - self_alt
            vehicle.log({'position_error': f'{dz_pos}'})


            
            

            # Calcul de la vitesse en z pour le maintien d'altitude
            vz_pos = kz_pos * (dz_pos + iz * prev_dz_pos)
            vehicle.log({'position vel': f'{vz_pos}'})

            # Calcul de l'erreur de distance
            prev_dx_d, prev_dy_d = dx_d, dy_d
            dx_d = d2 - 5

            # Calcul de la vitesse en x pour le suivi de distance
            vx_d = kx_d * (dx_d + ix_d * prev_dx_d)

            vehicle.log({'distance vel':f"{vx_d}"})
            
            vx = +vx_d
            vy = 0
            vz = vz_pos

    
            # Saturation des vitesses
            vx = max(min(vx, 1), -1)
            vz = max(min(vz, 0.5), -0.5)

            print(f"v:{vx},{vy},{vz}")

            # Envoi de la commande de vitesse au drone
            if not test:
                msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
                    0,  # time_boot_ms
                    vehicle.vehicle.target_system,
                    vehicle.vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,  # référentiel du drone
                    0b0000111111000111,  # ignore position, accel, yaw
                    0, 0, 0,  # position ignorée
                    vx, vy, vz,  # vitesses
                    0, 0, 0,  # accélérations ignorées
                    0, 0  # yaw ignoré
                )

                vehicle.vehicle.mav.send(msg)

            # La boucle s'éffectue à chaque réception de données du master (10 Hz)

except Exception as e:
    print("Error main loop:", e)