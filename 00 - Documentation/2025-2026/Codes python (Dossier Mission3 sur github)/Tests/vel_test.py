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

vehicle = Drone()
while vehicle.get_mode() != "STABILIZE":
    time.sleep(0.1)
while vehicle.get_mode() != "AUTO":
    time.sleep(0.1)
vehicle.set_mode("GUIDED")
vehicle.arm_and_takeoff(5)
vehicle.set_yaw(270)

print("Moving forward ref drone")
msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            vehicle.vehicle.target_system,
            vehicle.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # référentiel du drone
            0b0000111111000111,  # ignore position, accel, yaw
            0, 0, 0,  # position ignorée
            1, 0, 0,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )
vehicle.vehicle.mav.send(msg)
print("Moving forward NED")
time.sleep(3)
msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            vehicle.vehicle.target_system,
            vehicle.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # référentiel du drone
            0b0000111111000111,  # ignore position, accel, yaw mais pas yaw rate
            0, 0, 0,  # position ignorée
            1, 0, 0,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )
vehicle.vehicle.mav.send(msg)
time.sleep(3)
print("Moving forward ref drone ingore vz")
msg = vehicle.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            vehicle.vehicle.target_system,
            vehicle.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # référentiel du drone
            0b0000111111100111,  # ignore position, accel, yaw
            0, 0, 0,  # position ignorée
            1, 0, 0,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )
vehicle.vehicle.mav.send(msg)
time.sleep(3)
print("Landing...")
vehicle.set_mode("LAND")
time.sleep(10)
