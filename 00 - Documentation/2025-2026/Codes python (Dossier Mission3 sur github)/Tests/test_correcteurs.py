import argparse
from collections import namedtuple
import socket

import sys
import os
import time

# -----------------------------------------------------------------------------
# Script: test_correcteurs.py
# Description: vol pour tester un correcteur de position simple vers une position GPS fixe
# - Utilise une boucle de contrôle proportionnel/integral pour se diriger vers une position GPS cible.
# -----------------------------------------------------------------------------


# Paramètres P et I pour le correcteur
parser = argparse.ArgumentParser(description="UWB follower controller parameters")
parser.add_argument('-p', '--p', type=float, default=0.1, help='Proportional gain kx (default: 0.1)')
parser.add_argument('-i', '--i', type=float, default=0.01, help='Integral gain ix (default: 0.01)')
args = parser.parse_args()

kx = args.p
ky = args.p
kz = args.p

ix = args.i
iy = args.i
iz = args.i




sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])

# Initialisation du drone
vehicle = Drone()

print("alt =", vehicle.get_pose()[2])
vehicle.log({"alt": vehicle.get_pose()[2]})
dx = 0
dy = 0
dz = 0

vehicle.request_global_position_stream(rate_hz=10)


#Décollage automatique, à modifier pour inclure stabilize/auto avant de passer en guided pour plus de sécurité
vehicle.set_mode("GUIDED")
vehicle.arm_and_takeoff(10)
vehicle.set_yaw(0)


#Boucle du correcteur de position vers une position GPS fixe
#chateau d'angleterre 48.6300599	7.7892235, 150
#insa                 48.5824009, 7.7640712, 180
master_lat,master_lon,master_alt = 48.6300599, 7.7892235, 150
pos_master = Position(master_lat,master_lon,master_alt)
while True:

    # Position actuelle du drone
    self_lat,self_lon,self_alt = vehicle.get_pose()

    self_lat = self_lat/1e7
    self_lon = self_lon/1e7
    self_alt = self_alt
    vehicle.log({'self_position': f'{self_lat}, {self_lon}, {self_alt}'})

    # Calcul des erreurs de position
    if dx and dy and dz:
        prev_dx = dx
        prev_dy = dy
        prev_dz = dz
    else:
        prev_dx,prev_dy,prev_dz = Drone.pos_to_dist(self_lat,self_lon,self_alt,master_lat,master_lon,master_alt)
    dx,dy,dz = Drone.pos_to_dist(self_lat,self_lon,self_alt,master_lat,master_lon,master_alt)
    vehicle.log({'position_error': f'{dx}, {dy}, {dz}'})    

    #Vitesses maximales
    max_vx = 2
    max_vy = 2
    max_vz = 1
    

    # Calcul des vitesses à appliquer avec correcteur PI
    vx = max(min(kx*(dx + ix*prev_dx), max_vx), -max_vx)
    vy = max(min(ky*(dy + iy*prev_dy), max_vy), -max_vy)
    vz = max(min(kz*(dz + iz*prev_dz), max_vz), -max_vz)
    vehicle.log({'velocity': f'{vx}, {vy}, {vz}'})
    vehicle.move_velocity(vx,vy,-vz,0.09)