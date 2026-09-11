import argparse
from collections import namedtuple
import socket

import sys
import os
import time
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from pymavlink import mavutil
from Drone import Drone

# -----------------------------------------------------------------------------
# Script: uwb_follower_nobt.py
# Description: Contrôleur simple de suiveur utilisant des mesures UWB.
# - Lit les distances depuis un module UWB via la classe `Drone`.
# - Boucle de contrôle proportionnel/integral pour maintenir une distance cible (5 m) et une altitude de 1.5 m.
# -----------------------------------------------------------------------------


# Paramètres pour changer les gains P et I depuis la ligne de commande
parser = argparse.ArgumentParser(description="UWB follower controller parameters")
parser.add_argument('-p', '--p', type=float, default=0.3, help='Proportional gain kx (default: 0.1)')
parser.add_argument('-i', '--i', type=float, default=0.01, help='Integral gain ix (default: 0.01)')
args = parser.parse_args()

kx = args.p
ix = args.i


# Initialisation du drone et du module UWB
vehicle = Drone()
vehicle.start_uwb_serial()

#Attente du mode STABILIZE puis AUTO avant de passer en GUIDED et décoller
while vehicle.get_mode() != "STABILIZE":
    time.sleep(0.1)
while vehicle.get_mode() != "AUTO":
    time.sleep(0.1)
vehicle.set_mode("GUIDED")
vehicle.arm_and_takeoff(1.5)



# Attente du signal START par l'utilisateur (pas de Bluetooth ici)
try:
    while True:
        cmd = input("Type 'START' to continue: ").strip()
        if cmd.upper() == "START":
            print("START received, continuing...")
            break
except KeyboardInterrupt:
    print("\nInterrupted by user. Exiting.")
    sys.exit(0)

#Lancement de l'interruption propre du véhicule au changement de mode
vehicle.interrupt()

dist = None
change = False


#Thread pseudo-parallèle de mesure UWB
def measure_thread():
    global dist
    global change
    while True:
        d = vehicle.request_uwb_measure()
        print(f"Measured distance: {d} meters")
        if d is not None:
            msr = d.split(';')[1]
            d = float(msr.split('=')[1])
            dist = d
            change = True
        time.sleep(0.1)

# Lancement du thread de mesure UWB
import threading
import argparse
thread = threading.Thread(target=measure_thread, daemon=True)
thread.start()
print("Measuring distance...")



while not change:
    time.sleep(0.1)
distance_error = dist - 5.0  # cible à 5 mètres
alt = 1.5
dz = 0
while True:

    start_time = time.time()

    # Reception de l'altitude du drone
    alt = vehicle.get_alt_lidar()
     
    # Réinitialisation du flag de changement
    change = False
    vehicle.log({'dist': f'{dist} meters'})

    #Calcul de la commande de vitesse en x
    prev_dx = distance_error
    distance_error = dist - 5.0  # cible à 5 mètres
    max_vx = 1
    vx = max(min(kx*(distance_error + ix*prev_dx), max_vx), -max_vx)

    #Calcul de la commande de vitesse en z
    prev_dz = dz
    dz = alt - 1.5
    max_vz = 0.5
    vz = max(min(kx*(dz + ix*prev_dz), max_vz), -max_vz)

    #Log des vitesses
    vehicle.log({'vx': f'{vx} m/s'})
    vehicle.log({'vz': f'{vz} m/s'})

    #Envoi de la commande de vitesse au drone, pour les px4, il n'est pas nécessaire de "spammer" les commandes, le dernier ordre est maintenu
    msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            vehicle.vehicle.target_system,
            vehicle.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # référentiel du drone
            0b0000111111000111,  # ignore position, accel, yaw et vz ATT VZ
            0, 0, 0,  # position ignorée
            vx, 0, vz,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )
    vehicle.vehicle.mav.send(msg)

    # Attend une nouvelle mesure avant de réenvoyer
    while not change:
        pass
    # Log du temps de boucle
    vehicle.log({'loop_time': f'{(time.time() - start_time)*1000:.2f} ms'})
    