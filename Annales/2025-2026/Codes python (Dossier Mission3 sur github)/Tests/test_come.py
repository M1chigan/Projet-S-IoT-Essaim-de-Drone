import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone

vehicle = Drone()
vehicle = Drone()
vehicle.start_uwb_serial()

print(vehicle.who_am_i_uwb())
