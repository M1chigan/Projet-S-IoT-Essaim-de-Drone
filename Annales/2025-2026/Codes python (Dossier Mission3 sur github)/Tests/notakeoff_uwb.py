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
# Script: notakeoff_uwb.py
# Description: Test de contrôle en boucle fermé basé sur UWB sans décollage.
# - Initialise le module UWB via la classe `Drone`.
# - Lance un thread de mesure continu, les messages de déplacement ne sont pas envoyés au drone
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="UWB follower controller parameters")
parser.add_argument('-p', '--p', type=float, default=0.1, help='Proportional gain kx (default: 0.1)')
parser.add_argument('-i', '--i', type=float, default=0.01, help='Integral gain ix (default: 0.01)')
args = parser.parse_args()

kx = args.p
ix = args.i

vehicle = Drone()
vehicle.start_uwb_serial()
#vehicle.set_mode("AUTO")
#vehicle.set_mode("GUIDED")
#vehicle.arm_and_takeoff(2)
#vehicle.set_yaw(0)

dist = None
change = False

#Thread pseudo-parallèle de mesure UWB
def measure_thread():
    global dist
    global change
    while True:
        d = vehicle.request_uwb_measure()
        print(d)
        print(f"Measured distance: {d} meters")
        if d is not None:
            msr = d.split(';')[0]
            d = float(msr.split('=')[1])
            dist = d
            change = True
        time.sleep(0.5)

# Lancement du thread de mesure UWB
import threading
import argparse
thread = threading.Thread(target=measure_thread, daemon=True)
thread.start()
print("Measuring distance...")

while not change:
    time.sleep(0.1)
distance_error = dist - 5.0  # cible à 5 mètres
while True:
    start_time = time.time()
    change = False#réinitialise le flag de changement
    vehicle.log({'dist': f'{dist} meters'})#affiche la distance mesurée


    #obsolète :  calcule la commande de vitesse en x
    prev_dx = distance_error
    distance_error = dist - 5.0  # cible à 5 mètres
    max_vx = 1
    vx = max(min(kx*(distance_error + ix*prev_dx), max_vx), -max_vx)
    vehicle.log({'vx': f'{vx} m/s'}, print_=False)
    msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            vehicle.vehicle.target_system,
            vehicle.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # référentiel du drone
            0b0000111111000111,  # ignore position, accel, yaw
            0, 0, 0,  # position ignorée
            vx, 0, 0,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )
    

    # Attend une nouvelle mesure avant de réenvoyer
    while not change:
        time.sleep(0.01)
        vehicle.vehicle.mav.send(msg)
    vehicle.log({'loop_time': f'{(time.time() - start_time)*1000:.2f} ms'}, print_=False)
    