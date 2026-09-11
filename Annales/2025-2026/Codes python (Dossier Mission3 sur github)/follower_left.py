import argparse
import socket

import sys
import os
import time
# Dépend de la localisation du fichier utils.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
from Drone import Drone
# Instancie le véhicule une seule fois
vehicle = Drone()

vehicle.start_uwb_serial()

# Adresses MAC bluetooth des dispositifs connus (pas utilisés ici)
pcfixe_simeon = "60:45:2E:FC:BF:F1"
pi_simeon = "2C:CF:67:A2:82:5D"
portable_simeon = "4C:03:4F:E4:76:53"
pi_spacex = "DC:A6:32:49:79:E3"
pi_walle = "2C:CF:67:85:CB:51"
pi_ricardo = "2C:CF:67:77:35:81"

# Configuration réseau 
MASTER_IP = "10.42.0.54"
PORT = 5005


# Threa d'acquisition des mesures UWB
d1=0
d2=0
change = False
def measure_thread():
    global d1
    global d2
    global change
    while True:
        d = vehicle.request_uwb_measure()
        # Affiche la mesure brute reçue (utile en debug)
        print(f"Measured distance: {d} meters")
        if d is not None:
            msr = d.split(';')
            d1 = float(msr[0].split('=')[1])
            d2 = float(msr[1].split('=')[1])
            change = True
        time.sleep(0.1)


# Connexion au serveur maître
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((MASTER_IP, PORT))

# Attente mode STABILIZE
while vehicle.get_mode() != "STABILIZE":
    vehicle.log({"event": "En attente du mode STABILIZE"})

# Attente mode AUTO
while vehicle.get_mode() != "AUTO":
    vehicle.log({"event": "En attente du mode AUTO"})
vehicle.set_mode("GUIDED")

#Attente du message INIT du master
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        if "INIT" in data:
            client_socket.send("ACK".encode("utf-8"))
            break
except Exception as e:
    print("Error:", e)


#Phase d'initialisation: les drones mesures les distances UWB et en Position, pour en faire la moyenne et se caler dessus
print("INIT")
dists_c = []
dist_self = []
dlat = []
dlon = []
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        #Pas utilisé par le drone de gauche
        if "REQ" in data:
            pass
        else:
            try:
                #data_str = f"{d_C}:{lat:.6f}:{lon:.6f}:{alt}:{vx:.2f}:{vy:.2f}:{vz:.2f}"
                d_c,master_lat, master_lon, master_alt, master_vx, master_vy, master_vz = [float(x) for x in data.split(";")]
            except Exception:
                # unrecognized message; ignore
                continue
            lat,lon,alt = vehicle.get_pose()

            #Ajout des données pour faire la moyenne
            dlat.append(master_lat - lat/1e7)
            dlon.append(master_lon - lon/1e7)
            dists_c.append(d_c)
            if change:
                dist_self.append(d2)
                change = False
        # Si la taille des listes de mesures dépasse 20, on calcule la moyenne et on envoie l'ACK au master
        if len(dists_c) >= 20:
            client_socket.send("ACK".encode("utf-8"))
            break
except Exception as e:
    print("Error:", e)
print("INIT DONE")

#Calcul des moyennes des distances et positions
dlat_avg = sum(dlat)/len(dlat)
dlon_avg = sum(dlon)/len(dlon)
dists_c_avg = sum(dists_c)/len(dists_c)
dist_self_avg = sum(dist_self)/len(dist_self)

#Attente du message START du master
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



#Décollage et mise en position
vehicle.request_global_position_stream(rate_hz=10)
vehicle.set_mode("GUIDED")
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


#Démarrage du thread de mesure UWB
import threading
threading.Thread(target=measure_thread, daemon=True).start()

# Boucle principale de contrôle
try:
    # initialize previous errors to zero
    prev_dx_pos = prev_dy_pos = prev_dz_pos = 0.0
    dx_pos = dy_pos = dz_pos = 0.0
    prev_dx_d = prev_dy_d = 0.0
    dx_d = dy_d = 0.0
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        data = data.decode("utf-8")
        if "GPS" in data:
            # Server sent a GPS-not-available message; ignore
            continue
        elif "STOP" in data:
            vehicle.move_velocity(0, 0, -3, 1)
            vehicle.set_mode("LOITER")
            time.sleep(1)
            exit(0)
        else:
            try:
                #reception des données du master
                #data_str = f"{d_C}:{lat:.6f}:{lon:.6f}:{alt}:{vx:.2f}:{vy:.2f}:{vz:.2f}"
                d_c,master_lat, master_lon, master_alt, master_vx, master_vy, master_vz = [float(x) for x in data.split(";")]
            except Exception:
                # unrecognized message; ignore
                continue

            #Limitation des vitesses maximales par rapport à la vitesse du master
            max_vx = master_vx+1
            max_vy = master_vy+1
            max_vz = master_vz+0.5
            min_vx = master_vx-1
            min_vy = master_vy-1
            min_vz = master_vz-0.5

            #Calcul de la position cible du drone avec les offsets moyens
            lat_deg, lon_deg= master_lat+dlat_avg, master_lon+dlon_avg, master_alt
            self_lat, self_lon, self_alt = vehicle.get_pose(to = 0.2)
            self_lat = self_lat / 1e7
            self_lon = self_lon / 1e7
            vehicle.log({'master_position': f'{lat_deg}, {lon_deg}, {master_alt}'})
            vehicle.log({'self_position': f'{self_lat}, {self_lon}, {self_alt}'})
            vehicle.log({"dists":f"{d_c},{d2}"})

            # Calcul des erreurs de position
            prev_dx_pos, prev_dy_pos, prev_dz_pos = dx_pos, dy_pos, dz_pos
            dx_pos, dy_pos, dz_pos = vehicle.pos_to_dist(self_lat, self_lon, self_alt,
                                              lat_deg, lon_deg, master_alt)
            kx_pos = ky_pos = 0.1
            ix = iy = iz = 0.05
            kz_pos = 0.3
            iz_pos = 0.05
            vehicle.log({'position_error': f'{dx_pos}, {dy_pos}, {dz_pos}'})



            # Calcul des vitesses depuis les erreurs de position
            vx_pos = kx_pos * (dx_pos + ix * prev_dx_pos)
            vy_pos = ky_pos * (dy_pos + iy * prev_dy_pos)
            vz_pos = kz_pos * (dz_pos + iz * prev_dz_pos)
            vehicle.log({'position vel': f'{vx_pos}, {vy_pos}, {vz_pos}'})


            # Coefficients pour le contrôle en distance
            kx_d = 0.3
            ky_d = 0.3
            ix_d = 0.05
            iy_d = 0.05

            # Calcul de l'erreur de distance
            prev_dx_d, prev_dy_d = dx_d, dy_d
            dx_d = d2 - dist_self_avg
            dy_d = d_c - dists_c_avg
            # Calcul des vitesses depuis l'erreur de distance
            vx_d = kx_d * (dx_d + ix_d * prev_dx_d)
            vy_d = ky_d * (dy_d + iy_d * prev_dy_d)

            vehicle.log({'distance vel':f"{vx_d},{vy_d}"})
            

            # Somme des vitesses de position et de distance, ce code utilise deux sources d'information
            vx = vx_pos+vx_d
            vy = vy_pos+vy_d
            vz = vz_pos
            # Saturation des vitesses
            vx = max(min(vx, max_vx), min_vx)
            vy = max(min(vy, max_vy), min_vy)
            vz = max(min(vz, max_vz), min_vz)

            print(f"v:{vx},{vy},{vz}")

            vehicle.move_velocity(vx, vy, -vz, 0.095)

except Exception as e:
    print("Error:", e)