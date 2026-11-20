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

try:
    vehicle = Drone(path = "/dev/ttyACM1")
except:
    print("default")
    vehicle = Drone()
print(vehicle.get_alt_lidar())
lat,lon,alt,vx,vy,vz = vehicle.get_pose_vel()
lat, lon, alt = vehicle.get_pose()
print(f"Lat: {lat}, Lon: {lon}, Alt: {alt}, Vx: {vx}, Vy: {vy}, Vz: {vz}")