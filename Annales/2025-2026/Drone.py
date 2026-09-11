from curses.ascii import alt
from pymavlink import mavutil
import time
import datetime
from collections import namedtuple
import math
from math import sin, cos, sqrt, atan2, radians
import os
import json
from datetime import datetime
import inspect
import cv2
import threading
import pymap3d as pm
import numpy as np
import serial
import subprocess

import platform

if platform.machine() == "aarch64" and "raspberrypi" in platform.uname().node.lower():
    # Test if Pi 5 (model name contains 'Raspberry Pi 5')
    if "raspberry pi 5" in platform.uname().machine.lower() or "raspberry pi 5" in platform.uname().version.lower():
        from ultralytics import YOLO
        from picamera2 import Picamera2
    else:
        YOLO = None
else:
    YOLO = None

Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])

class Drone:

    def __init__(self, path="/dev/ttyACM0", feedback=False, gps_rate = 10):
        self.path = path
        self.vehicle = None
        self.feedback = None
        self.camera_rpi4 = None
        self.camera_rpi5 = None
        self.connect(feedback, gps_rate = gps_rate)


        # Programme thomas
        self.kp_atterrissage = 0
        self.kd_atterrissage = 0.0002
        self.ki_atterrissage = 0.000001
        self.coefficient_kp_atterrissage = 0.5

        self.kp_hauteur = 1/3

        self.erreurIntegraleEst_atterrissage = 0
        self.erreurIntegraleNord_atterrissage = 0
        self.erreurAnterieureEst_atterrissage = 0
        self.erreurAnterieureNord_atterrissage = 0

#----------------------------------------------
# Fonctions générales
# ---------------------------------------------

    # Fonction pour se connecter au drone
    def connect(self, feedback, gps_rate = 10):
        try:
            vehicle = mavutil.mavlink_connection(self.path)
            vehicle.wait_heartbeat()
            print("Drone connecté.")

            # Demande les datastreams basiques
            vehicle.mav.request_data_stream_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

            self.vehicle = vehicle
            
            # Demande automatiquement le flux GLOBAL_POSITION_INT à 10 Hz (100ms)
            self.request_global_position_stream(rate_hz=gps_rate)
            self.log({"event":"Drone connecté."}, log_name="default.json")

            if feedback:
                feedback = mavutil.mavlink_connection(self.path, source_system=1)
                feedback.wait_heartbeat()
                self.feedback = feedback
                self.print_mavlink("Feedback connecté.")
                self.log({"event":"Feedback connecté."}, log_name="default.json")
        except:
            raise
    
    # Fonctions pour gérer les interruptions (changement de mode, niveau de batterie...)
    def interrupt(self):
        # Ensure we don't start multiple interrupt threads
        if hasattr(self, "_interrupt_thread") and getattr(self, "_interrupt_thread") is not None:
            return
        interrupt_thread = threading.Thread(target=self.interrupt_function, args=(None,), daemon=True)
        interrupt_thread.start()
        self._interrupt_thread = interrupt_thread
        print("Thread interruption started")
    def interrupt_function(self, exit_function = None):
        while True:
            mode = self.get_mode(log = False)
            if mode != "GUIDED":
                self.log({'Error': f"Flight mode changed from GUIDED to {mode}. Interrupting script."})
                if exit_function:
                    exit_function()
                os._exit(1)
            time.sleep(1)

    # Fonction pour enregistrer des logs
    def log(self, data: dict = None, log_name: str = "log.json", print_ = True):
        # Obtenir la date et l'heure actuelles
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S-%f")[:-3]  # include milliseconds

        # Créer le chemin du dossier de log
        log_dir = os.path.join("Logs", date_str)
        os.makedirs(log_dir, exist_ok=True)

        # Chemin complet du fichier log JSON
        log_file_path = os.path.join(log_dir, log_name)

        # Récupérer le nom du fichier appelant (si possible)
        try:
            caller_frame = inspect.stack()[1]
            caller_filename = os.path.basename(caller_frame.filename)
        except Exception:
            caller_filename = "unknown_file"

        # Préparer la donnée à enregistrer
        log_entry = {
            "file": caller_filename,
            "time": time_str,
            "data": data if data else {}
        }

        # Lire l'existant si le fichier existe
        if os.path.exists(log_file_path):
            with open(log_file_path, "r", encoding="utf-8") as f:
                try:
                    logs = json.load(f)
                except json.JSONDecodeError:
                    logs = []
        else:
            logs = []

        # Ajouter la nouvelle entrée
        logs.append(log_entry)

        # Écrire dans le fichier JSON
        with open(log_file_path, "w", encoding="utf-8") as f:
            json.dump(logs, f, ensure_ascii=False, indent=4)
        if print_:
            print(log_entry)

    # Fonction pour envoiyer un message texte dans Mission Planner ou QGroundControl
    def print_mavlink(self, text: str, severity: int = 6):
        """
        MAV_SEVERITY_ALERT = 1

        MAV_SEVERITY_ERROR = 3

        MAV_SEVERITY_WARNING = 4

        MAV_SEVERITY_INFO = 6 (par défaut)
        """
        if not self.feedback:
            print("Erreur : feedback non connecté")
            return

        # Convertir en texte si ce n’est pas déjà une chaîne
        if not isinstance(text, str):
            text = str(text)
        
        # Limite à 50 caractères (limite MAVLink pour STATUSTEXT)
        text = text[:50]

        # Envoi du message
        print(text)
        self.feedback.mav.statustext_send(severity, text.encode())

    # Fonction définir la caméra (Pi 5)
    def set_camera_rpi5(self, camera, config=None):
        if config:
            camera.configure(config)
        else:
            config = camera.create_video_configuration(main={"size": (1920, 1080)})
            camera.configure(config)
        self.camera_rpi5 = camera
        self.log({"event":"Caméra rpi5 définie."}, log_name="default.json")

    # Fonction pour prendre une photo avec la caméra (Pi 4)
    def take_photo_rpi4(self, filename="Images/capture_test_camera_rpi4.jpg"):
        print("Tentative de capture via rpicam-jpeg...")
        
        # On s'assure que le dossier Images existe
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        # Commande système brute
        cmd = ["rpicam-jpeg", "-n", "-t", "500", "--immediate", "-o", "-"]
        
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            
            if res.returncode == 0:
                # Transformation des octets en image OpenCV
                data = np.frombuffer(res.stdout, dtype=np.uint8)
                image = cv2.imdecode(data, cv2.IMREAD_COLOR)
                
                if image is not None:
                    cv2.imwrite(filename, image)
                    print(f"✅ SUCCÈS : Photo enregistrée dans {filename}")
                    return True
                else:
                    print("❌ ERREUR : Décodage OpenCV échoué.")
            else:
                print(f"❌ ERREUR Système : {res.stderr.decode()}")
        except Exception as e:
            print(f"❌ ERREUR Critique : {e}")
        return False

    # Fonction pour prendre une photo avec la caméra (Pi 5)
    def take_photo_rpi5(self, filename: str = "Images_camera_rpi5", delai=0.3):
        # Il faut définir la caméra pour utiliser cette fonction
        # Exemple : 
        # picam2 = Picamera2() 
        # config = picam2.create_video_configuration(main={"size": (1920, 1080)})
        # picam2.configure(config)
        # vehicle.set_camera_rpi5(picam2)

        if self.camera_rpi5 is None:
            raise RuntimeError("Pi 5 camera is not defined")

        # Crée un dossier avec la date du jour
        date_str = datetime.now().strftime("%Y-%m-%d")
        img_dir = os.path.join("Images_rpi5", date_str)
        os.makedirs(img_dir, exist_ok=True)

        # Capture une image
        self.camera_rpi5.start()
        # Si c'est une caméra avec autofocus on l'active
        controls_to_set = {}
        if "AfMode" in self.camera_rpi5.camera_controls:
            controls_to_set["AfMode"] = 1
        if "AfTrigger" in self.camera_rpi5.camera_controls:
            controls_to_set["AfTrigger"] = 0
        if controls_to_set:
            self.camera_rpi5.set_controls(controls_to_set)
            time.sleep(delai)
        frame = self.camera_rpi5.capture_array()
        self.camera_rpi5.stop()

        if frame is None:
            raise RuntimeError("Unable to capture an image with the Pi 5 camera")
            
        # Conversion en BGR (OpenCV attend BGR)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Génération d’un nom unique pour la photo
        i = 0
        while os.path.exists(f"{img_dir}/{filename}_{i}.jpg"):
            i += 1
        filename = f"{img_dir}/{filename}_{i}.jpg"
        cv2.imwrite(filename, frame_bgr)
        print(f"Photo enregistrée : {filename}")
        self.log({"event":"Photo rpi5 enregistrée."}, log_name="default.json")

    # Fonction pour enregistrer une photo avec la caméra (Pi 5)
    def save_image_rpi5(self, filename: str = "Images/Save_image_rpi5.jpg", delai=0.3):
        # Il faut définir la caméra pour utiliser cette fonction
        # Exemple : 
        # picam2 = Picamera2() 
        # config = picam2.create_video_configuration(main={"size": (1920, 1080)})
        # picam2.configure(config)
        # vehicle.set_camera_rpi5(picam2)

        if self.camera_rpi5 is None:
            raise RuntimeError("Pi 5 camera is not defined")

        # Capture une image
        self.camera_rpi5.start()
        # Si c'est une caméra avec autofocus on l'active
        controls_to_set = {}
        if "AfMode" in self.camera_rpi5.camera_controls:
            controls_to_set["AfMode"] = 1
        if "AfTrigger" in self.camera_rpi5.camera_controls:
            controls_to_set["AfTrigger"] = 0
        if controls_to_set:
            self.camera_rpi5.set_controls(controls_to_set)
            time.sleep(delai)
        frame = self.camera_rpi5.capture_array()
        self.camera_rpi5.stop()

        if frame is None:
            raise RuntimeError("Unable to capture an image with the Pi 5 camera")
            
        # Conversion en BGR (OpenCV attend BGR)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Génération d’un nom unique pour la photo
        cv2.imwrite(filename, frame_bgr)
        print(f"Photo enregistrée : {filename}")
        self.log({"event":"Photo rpi5 enregistrée."}, log_name="default.json")

    # Fonction pour prendre une photo avec la caméra (Pi 4)
    def get_image_rpi4(self):
        # Il faut définir la caméra pour utiliser cette fonction
        # Exemple : cap = cv2.VideoCapture(0) puis vehicle.set_camera_rpi4(self, cap)

        if self.camera_rpi4 is None:
            raise RuntimeError("Pi 4 camera is not defined")

        ret, frame = self.camera_rpi4.read()

        if not ret:
            raise RuntimeError("Unable to capture an image with the Pi 4 camera")
        
        self.log({"event":"Image rpi4 obtenue."}, log_name="default.json")

        self.camera_rpi4.release()

        return frame

    # Fonction pour prendre une photo avec la caméra (Pi 5)
    def get_image_rpi5(self, delai=0.3):
        # Il faut définir la caméra pour utiliser cette fonction
        # Exemple : 
        # picam2 = Picamera2() 
        # config = picam2.create_video_configuration(main={"size": (1920, 1080)})
        # picam2.configure(config)
        # vehicle.set_camera_rpi5(picam2)

        if self.camera_rpi5 is None:
            raise RuntimeError("Pi 5 camera is not defined")

        self.camera_rpi5.start()
        # Si c'est une caméra avec autofocus on l'active
        controls_to_set = {}
        if "AfMode" in self.camera_rpi5.camera_controls:
            controls_to_set["AfMode"] = 1
        if "AfTrigger" in self.camera_rpi5.camera_controls:
            controls_to_set["AfTrigger"] = 0
        if controls_to_set:
            self.camera_rpi5.set_controls(controls_to_set)
            time.sleep(delai)
        frame = self.camera_rpi5.capture_array()
        self.camera_rpi5.stop()

        if frame is None:
            raise RuntimeError("Unable to capture an image with the Pi 5 camera")

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        self.log({"event":"Image rpi5 obtenue."}, log_name="default.json")

        return frame_bgr

    # Fonction pour changer le mode de vol du drone
    def set_mode(self, mode_name):
        mode_id = self.vehicle.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"Mode {mode_name} inconnu !")
            return

        # Envoi du message MAVLink pour changer de mode
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Attendre la confirmation
        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
            if msg.custom_mode == mode_id:
                break
        
        self.log({"event":"Changement de mode effectué.", "mode": mode_name}, log_name="default.json")

    # Fonction pour récupèrer le mode de vol du drone
    def get_mode(self, log = True):
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
        #Teste le message et attend de bien recevoir le bon type de systeme (pas GCS par exemple)
        while not msg or msg.type != 2: # MAV_TYPE_QUADROTOR
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)

        mode_id = msg.custom_mode
        mode_name = None

        for name, mode in self.vehicle.mode_mapping().items():
            if mode == mode_id:
                mode_name = name
                break
        if log:
            if mode_name:
                print(f"Mode actuel : {mode_name}")
            else:
                print(f"Mode inconnu (ID: {mode_id})")

            self.log({"event":"Mode obtenue.", "mode":mode_name}, log_name="default.json")

        return mode_name

    # Fonction pour attérir
    def land(self):
        print("Switching to LAND mode...")
        self.set_mode("LAND")
        print("Drone is landing")
        self.log({"event":"Atterissage du drone."}, log_name="default.json")

    # Fonction pour retourner au point de décollage
    def return_to_launch(self):
        print("Switching to RTL (Return to Launch) mode...")
        self.set_mode("RTL")
        print("Drone is returning to launch point")
        self.log({"event":"Retour au point de décollage."}, log_name="default.json")

    # Fonction permettant de définir l'orientation (yaw) du drone
    def set_yaw(self, heading, relative=False, clockwise=True):
        # =1 yaw relatif à la direction actuelle, =0 yaw absolu par rapport au nord
        is_relative = 1 if relative else 0

        # =1 for clockwise, =-1 for counterclockwise
        is_clockwise = 1 if clockwise else -1

        # Envoi de la commande CONDITION_YAW via command_long_encode
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,               # ID du système cible
            self.vehicle.target_component,            # ID du composant cible
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # ID de la commande
            0,                                  # confirmation
            heading,                            # param 1 : yaw en degrés
            0,                                  # param 2 : vitesse de rotation en deg/s (0 = vitesse par défaut)
            is_clockwise,                                  # param 3 : direction (-1 = ccw, 1 = cw)
            is_relative,                         # param 4 : 1 = relatif, 0 = absolu
            0, 0, 0                              # param 5 à 7 non utilisés
        )

        # Attendre que le changement de yaw soit effectué
        if relative:
            current_yaw = self.get_yaw()
            if clockwise:
                heading = (current_yaw + heading) % 360
            else:
                heading = (current_yaw - heading) % 360
        self.wait_for_yaw_change(heading, clockwise=clockwise)

        self.log({"event":"Changement de yaw effectué.", "yaw":heading, "relative":relative, "clockwise":clockwise}, log_name="default.json")

    # Fonction pour récupérer l'orientation (yaw) actuelle du drone en degrés
    def get_yaw(self):
        # Récupération du message ATTITUDE
        msg = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        if msg:
            yaw = math.degrees(msg.yaw)       # conversion en degrés
            yaw = (yaw + 360) % 360           # normalisation entre 0 et 360 degrés
            self.log({"event":"Yaw obtenue.", "yaw":yaw}, log_name="default.json")
            return yaw
        else:
            raise RuntimeError("Error: unable to retrieve drone orientation (yaw)")

    # Fonction pour récupérer l'attitude (roll, pitch, yaw) actuelle du drone en degrés
    def get_attitude(self):
        msg = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=3)
        if msg:
            self.log({"event":"Attitude obtenue.","roll":math.degrees(msg.roll), 
                      "pitch":math.degrees(msg.pitch), "yaw":math.degrees(msg.yaw)}, log_name="default.json")
            return math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
        return None

    # Fonction pour récupérer la position GPS
    def get_pose(self, to = 3):
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=to)
        if msg:
            #self.log({"event":"Pose obtenue.", "lat":msg.lat, "lon":msg.lon, "alt":msg.alt*0.001}, log_name="default.json")
            return msg.lat, msg.lon, msg.alt*0.001   #lat et lon en degE7 et alt en mm
        if not msg:
            raise RuntimeError("GPS position is missing or unavailable")
        

    def get_pose_vel(self, to = 3):
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=to)
        if msg:
            #self.log({"event":"Pose obtenue.", "lat":msg.lat, "lon":msg.lon, "alt":msg.alt*0.001}, log_name="default.json")
            return msg.lat, msg.lon, msg.alt*0.001, msg.vx*0.001, msg.vy*0.001, msg.vz*0.001   #lat et lon en degE7 et alt en mm
        if not msg:
            raise RuntimeError("GPS position is missing or unavailable")
        
   

    # Fonction pour récupérer l'altitude via le Lidar
    def get_alt_lidar(self):
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR)
        msg = self.vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=3)
        if not msg:
            raise RuntimeError("Distance sensor data is missing or unavailable")
        alt = msg.current_distance / 100.0  # conversion en mètres
        #self.log({"event":"Altitude (lidar) obtenue.", "alt":alt}, log_name="default.json")
        return alt

    # Fonction pour récupérer l'altitude via GLOBAL_POSITION_INT
    def get_alt(self):
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if not msg:
            raise RuntimeError("altitude GLOBAL_POSITION_INT data is missing or unavailable")
        alt = msg.alt / 1000 # conversion mm to m
        self.log({"event":"Altitude obtenue.", "alt":alt}, log_name="default.json")
        return alt

    # Fonction pour récupérer le niveau de la batterie
    def get_bat(self):
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
        if not msg:
            raise RuntimeError("Battery status is missing or unavailable")
        bat = msg.voltage_battery / 1000  # conversion en volts
        self.log({"event":"Batterie Obtenue.", "bat":bat}, log_name="default.json")
        return bat

    # Fonction pour vérifier que le GPS est prêt
    def gps_fix_ok(self):
        msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
        result = msg.fix_type >= 3 if msg else False
        self.log({"event":"GPS fix ok.", "gps_fix":result}, log_name="default.json")
        return result

    # Fonction pour vérifier que les capteurs fonctionnent, donc que le drone est utilisable
    def is_armable(self):
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=True)
        prearm_check = msg.onboard_control_sensors_health
        result = prearm_check > 0
        self.log({"event":"Drone armable.", "is_armable":result}, log_name="default.json")
        return result

    # Fonction pour savoir si le drone est armé
    def is_armed(self):
        result = self.vehicle.motors_armed()
        self.log({"event":"Drone armé.", "is_armed":result}, log_name="default.json")
        return result

    # Fonction pour armer et décoller
    def arm_and_takeoff(self, target_alt):
        # Vérifie que le drone est armable
        print("Basic pre-arm checks")
        if not self.is_armable():
            raise RuntimeError("Pre-arm checks failed: drone is not armable")
        
        time.sleep(1)

        # Arme le drone
        print("Arming motors...")
        self.vehicle.arducopter_arm()

        # Vérifie que le drone est armé
        self.vehicle.motors_armed_wait()
        print("Drone armed")

        # Démarre le décollage
        print("Taking off")
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                    0, 0, 0, 0, 0, 0, target_alt)
        print(f"Takeoff to {target_alt}m")
        
        last_alt = 0
        last_change_time = time.time()
        timeout_duration = 3.0  # secondes
        tolerance = 0.1         # marge de 10cm pour considérer que l'altitude a changé

        while True:
            msg = self.vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
            alt = msg.current_distance / 100.0 # Conversion en mètres
            
            current_time = time.time()
            print(f"Altitude : {alt:.2f}m")

            if alt >= target_alt * 0.95:
                print("Target altitude reached")
                break
            if abs(alt - last_alt) > tolerance:
                last_alt = alt
                last_change_time = current_time
            else:
                if (current_time - last_change_time) > timeout_duration:
                    print(f"ERREUR : Décollage bloqué à {alt:.2f}m pendant 3s !")
                    break
        
        self.log({"event":"Le drone s'est armé et a décollé."}, log_name="default.json")

    # Fonction pour armer le drone
    def arm(self):
        # Vérifie que le drone est armable
        print("Basic pre-arm checks")
        if not self.is_armable():
            raise RuntimeError("Pre-arm checks failed: drone is not armable")
        
        time.sleep(1)

        # Arme le drone
        print("Arming motors...")
        self.vehicle.arducopter_arm()

        # Vérifie que le drone est armé
        self.vehicle.motors_armed_wait()
        self.log({"event":"Le drone s'est armé."}, log_name="default.json")

    # Fonction pour désarmer le drone
    def disarm(self):
        if not self.vehicle.motors_armed():
            print("Drone is already disarmed")
            return

        print("Disarming motors...")
        self.vehicle.arducopter_disarm()
        time.sleep(1)
        """
        # Attendre que le drone se désarme
        timeout = 10  # seconds
        start_time = time.time()
        print("self.is_armed() : "+str(self.is_armed()))
        while self.is_armed():
            if time.time() - start_time > timeout:
                raise RuntimeError("Failed to disarm the drone")
            time.sleep(0.5)"""

        self.log({"event":"Le drone s'est désarmé."}, log_name="default.json")

    # Fonction pour décoller
    def takeoff(self, target_alt):
        # Vérifie que le drone est armé
        #if not self.vehicle.motors_armed():
        #    raise RuntimeError("Drone not armed")
        while (not self.is_armed):
            time.sleep(0.1)

        print("Drone armed")
        # Démarre le décollage
        print("Taking off")
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                    0, 0, 0, 0, 0, 0, target_alt)
        print(f"Takeoff to {target_alt}m")

        # Attente de l'altitude cible
        while True:
            msg = self.vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
            alt = msg.current_distance / 100.0 # Conversion en mètres
            print(f"Altitude : {alt:.1f}m")
            if alt >= target_alt * 0.95: # Seuil à 95%
                print("Target altitude reached")
                break

        self.log({"event":"Le drone a décollé."}, log_name="default.json")

    # Fonction pour changer l'altitude
    def change_altitude(self, target_alt):
        # On récupère la dernière position GPS connue
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is None:
            raise RuntimeError("GPS position is missing or unavailable")
        
        lat = msg.lat
        lon = msg.lon
        print(lat/1e7, lon/1e7)
        
        # On envoie la commande de changement d'altitude
        type_mask = 0b110111111000
        self.vehicle.mav.set_position_target_global_int_send(

                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat),
                int(lon),
                target_alt,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
        print(f"Moving to {target_alt:.1f}m")

        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = msg.relative_alt / 1000.0
            print(f"Current altitude: {alt:.1f} m")
            if abs(alt - target_alt) <= 1:
                print(f"Target altitude reached")
                time.sleep(1)
                break
        
        self.log({"event":"Le drone a changé d'altitude.", "alt":target_alt}, log_name="default.json")

    # Fonction pour attendre pendant le déplacement du drone vers une position
    def wait_until_arrival(self, target_lat, target_lon, target_alt, tolerance_m=0.5):
        iteration = 0
        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if not msg:
                raise RuntimeError("GPS position is missing or unavailable")
            curr_lat = msg.lat / 1e7
            curr_lon = msg.lon / 1e7

            alt_msg = self.vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=3)
            if alt_msg is None:
                raise RuntimeError("DISTANCE SENSOR position is missing or unavailable")
            curr_alt = alt_msg.current_distance / 100.0

            d_lat = curr_lat - target_lat
            d_lon = curr_lon - target_lon
            d_alt = curr_alt - target_alt

            dist = math.sqrt((d_lat * 111139)**2 + (d_lon * 111139)**2 + d_alt**2)
            if dist < tolerance_m:
                print("Target position reached")
                break
            time.sleep(0.5)
            if dist < 10:
                iteration += 1
            if iteration > 20 :
                break
                #self.goto(target_lat, target_lon, target_alt)
                #self.wait_until_arrival(target_lat, target_lon, target_alt, tolerance_m+1)
        self.log({"event":"Le drone est arrivé à sa destination."}, log_name="default.json")

    # Fonction pour attendre que le drone ait terminé sa rotation
    def wait_for_yaw_change(self, target_degrees, clockwise=True, tolerance=2):
        while True:
            current_yaw = self.get_yaw() % 360

            if clockwise:
                delta = (target_degrees - current_yaw) % 360
            else:
                delta = -((current_yaw - target_degrees) % 360)

            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360

            if abs(delta) <= tolerance:
                break
        
        self.log({"event":"Le drone a changé d'orientation."}, log_name="default.json")

    # Fonction pour déplacer le drone vers un point GPS
    def goto(self, target_lat, target_lon, target_alt):
        type_mask = 0b110111111000

        self.vehicle.mav.set_position_target_global_int_send(

                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(target_lat * 1e7),
                int(target_lon * 1e7),
                target_alt,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
        
        self.log({"event":"Le drone se déplace vers un point gps."}, log_name="default.json")

    # Fonction pour tourner le drone vers la gauche d'un angle relatif en degrés
    def turn_left(self, degrees):
        if degrees <= 0:
            raise ValueError("The angle must be positive")
        print(f"Turning left {degrees}°")
        self.log({"event":"Le drone tourne à gauche."}, log_name="default.json")
        self.set_yaw(degrees, relative=True, clockwise=False)

    # Fonction pour tourner le drone vers la droite d'un angle relatif en degrés
    def turn_right(self, degrees):
        if degrees <= 0:
            raise ValueError("The angle must be positive")
        print(f"Turning right {degrees}°")
        self.log({"event":"Le drone tourne à droite."}, log_name="default.json")
        self.set_yaw(degrees, relative=True, clockwise=True)
    
    # Fonction pour déplacer le drone avec des vitesses relatives pur une durée donnée
    def move_velocity(self, vx, vy, vz, duration):

        #vx : vitesse vers l'avant (m/s) (négatif = arrière)
        #vy : vitesse vers la droite (m/s) (négatif = gauche)
        #vz : vitesse verticale (m/s, négatif = monter)
        #duration : durée du mouvement en secondes

        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # référentiel du drone
            0b0000111111000111,  # ignore position, accel, yaw
            0, 0, 0,  # position ignorée
            vx, vy, vz,  # vitesses
            0, 0, 0,  # accélérations ignorées
            0, 0  # yaw ignoré
        )

        self.vehicle.mav.send(msg)
        end_time = time.time() + duration
        while time.time() < end_time:
            self.vehicle.mav.send(msg)
            time.sleep(0.001)

    # Fonction pour déplacer le drone avec des vitesses relatives
    def set_velocity(self, vx, vy, vz):

        #vx : vitesse vers l'avant (m/s) (négatif = arrière)
        #vy : vitesse vers la droite (m/s) (négatif = gauche)
        #vz : vitesse verticale (m/s, négatif = monter)

        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame (Velocity and Acceleration are in NED frame)
            0x0DC7,  # type_mask (ignore pos | ignore acc)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,
            # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)
            # Envoie de la consigne de vitesse au drone 

    # Fonction pour effectuer un déplacement vers l'avant
    def move_forward(self, speed=1, duration=1):
        self.move_velocity(speed, 0, 0, duration)
        self.log({"event":"Le drone avance."}, log_name="default.json")

    # Fonction pour effectuer un déplacement vers le haut
    def move_up(self, speed=1, duration=1):
        self.move_velocity(0, 0, -speed, duration)
        self.log({"event":"Le drone monte."}, log_name="default.json")

    # Fonction pour effectuer un déplacement vers le bas
    def move_down(self, speed=1, duration=1):
        self.move_velocity(0, 0, speed, duration)
        self.log({"event":"Le drone descend."}, log_name="default.json")

    # Fonction pour faire un bip sonore
    def bip(self, repetitions=1, duration_s=0.5):
        if self.vehicle is None:
            print("Erreur: L'objet de connexion MAVLink (self.vehicle) n'est pas initialisé.")
            return

        try:
            tempo = int(60 / duration_s)
        except ZeroDivisionError:
            print("Erreur: La durée ne peut pas être zéro. Utilisation de 0.25s par défaut.")
            tempo = 240  # Tempo pour 0.25s

        # Le Tempo MAVLink doit être compris entre 32 et 255 (bornage)
        tempo = max(32, min(255, tempo))

        if tempo == 255 and duration_s > 60 / 255:
            print(f"Durée trop courte demandée. Tempo borné à 255 (0.24s).")
        elif tempo == 32 and duration_s < 60 / 32:
            print(f"Durée trop longue demandée. Tempo borné à 32 (1.875s).")

        # Limite de sécurité pour la longueur de la chaîne PLAY_TUNE (max 30)
        repetitions = min(repetitions, 5)

        # MFTxxxL4 : Mode Foreground (MF), T=Tempo calculé, L=Longueur 4 (noire)
        pattern_prefix = f"MFT{tempo}L4"

        # Motif de répétition : Note 'C' (Do) à l'octave 5 + Pause de même durée (P4)
        note_pattern = "O5C P4"

        # Construction de la chaîne complète
        full_pattern = pattern_prefix + (note_pattern * repetitions)

        # Troncation finale à 30 caractères (limite du champ 'tune')
        tune_part1 = full_pattern[:30]
        tune_part2 = ""  # Champ d'extension vide (Legacy PLAY_TUNE)

        self.vehicle.mav.play_tune_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            tune_part1.encode('ascii'),
            tune_part2.encode('ascii')
        )

        self.log({"event":"Bip sonore effectué."}, log_name="default.json")

    # Fonction pour jouer une mélodie MML pour le hackathon drone
    def bip_melodie(self, mml_segment):
        if self.vehicle is None:
            print("Erreur: L'objet de connexion MAVLink (self.vehicle) n'est pas initialisé.")
            return

        # Troncation à 30 caractères (limite du champ 'tune' de MAVLink)
        tune_part1 = mml_segment[:30].encode('ascii')
        tune_part2 = "".encode('ascii')  # Champ d'extension vide

        self.vehicle.mav.play_tune_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            tune_part1,
            tune_part2
        )

    # Fonction pour demander l'accès aux paramètres du drone
    def request_params(self):
        if self.vehicle is None:
            print("Erreur: L'objet de connexion MAVLink (self.vehicle) n'est pas initialisé.")
            return

        self.vehicle.mav.param_request_list_send(
            self.vehicle.target_system,
            1  # target_component (1 = autopilot)
        )

    # Fonction pour lire les paramètres demandés au drone 
    def read_params(self, nr):
        if self.vehicle is None:
            print("Erreur: L'objet de connexion MAVLink (self.vehicle) n'est pas initialisé.")
            return
        list_params = {}
        count = 0
        while count < len(nr):
            msg = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if msg:
                param_id = msg.param_id.strip('\x00')
                param_value = msg.param_value
                param_type = msg.param_type
                list_params[param_id] = (param_value, param_type)
                if param_id in nr:
                    print(f"Paramètre: {param_id}, Valeur: {param_value}, Type: {param_type}")
                    count += 1
            else:
                print("Aucun message PARAM_VALUE reçu dans le délai imparti.")
                break
        if list_params:
            return list_params
        else:
            print("Aucun paramètre reçu.")
            return
            print("Aucun message PARAM_VALUE reçu dans le délai imparti.")
            return 

    def request_message(self, message):
        print("request msg")
        if not self.vehicle:
            print("Erreur: Drone non connecté")
            return
        if not isinstance(message, int):
            print('Erreur (request_message): message pas int')
            return
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,  # confirmation
            message,  # param1: message ID
            0, 0, 0, 0, 0, #non utilisé
            2 # response target: broadcast
        )
#----------------------------------------------
# Fonctions spécifiques à la mission 1
# ---------------------------------------------

    # Fonction pour démarrer le flux vidéo de la caméra rpi5
    def start_camera_rpi5(self, config=None, delai=2):
        """
        Initialise et démarre la caméra Pi 5 une seule fois.
        Laisse le temps à l'exposition automatique de se stabiliser.
        """
        if hasattr(self, "_camera_started") and self._camera_started:
            self.log({"event": "Camera déjà démarrée"}, log_name="default.json")
            return

        if not self.camera_rpi5 :
            camera = Picamera2()
            if config:
                camera.configure(config)
            else:
                config = camera.create_video_configuration(main={"size": (1920, 1080)})
                camera.configure(config)
            self.camera_rpi5 = camera
        
        self.camera_rpi5.start()

        # Laisser l’exposition automatique s’ajuster
        time.sleep(delai)

        self._camera_started = True
        self.log({"event": "Camera démarrée"}, log_name="default.json")
    
    # Fonction pour arrêter le flux vidéo de la caméra rpi5
    def stop_camera_rpi5(self):
        """
        Stoppe proprement la caméra et libère les ressources.
        """
        if hasattr(self, "_camera_started") and self._camera_started:
            try:
                self.camera_rpi5.stop()
                self._camera_started = False
                self.log({"event": "Camera arrêtée"}, log_name="default.json")
            except Exception as e:
                self.log({"error": f"Erreur à l'arrêt de la caméra : {e}"}, log_name="default.json")
        else:
            self.log({"event": "Camera déjà arrêtée ou non initialisée"}, log_name="default.json")

    # Fonction pour capturer une image du flux vidéo de la caméra rpi5
    def capture_camera_rpi5(self, delai=0.3):
        """
        Capture une image depuis la caméra déjà démarrée.
        Ne stoppe pas la caméra après la capture.
        """
        if not hasattr(self, "_camera_started") or not self._camera_started:
            raise RuntimeError("La caméra Pi 5 n’est pas démarrée. Appeler start_camera_rpi5() d'abord.")

        # Gestion autofocus si disponible
        controls_to_set = {}
        if "AfMode" in self.camera_rpi5.camera_controls:
            controls_to_set["AfMode"] = 1  # Mode auto
        if "AfTrigger" in self.camera_rpi5.camera_controls:
            controls_to_set["AfTrigger"] = 0  # Déclenchement autofocus
        if controls_to_set:
            self.camera_rpi5.set_controls(controls_to_set)
            time.sleep(delai)

        # Capture l’image directement depuis le flux
        frame = self.camera_rpi5.capture_array()
        if frame is None:
            raise RuntimeError("Impossible de capturer une image avec la caméra Pi 5")

        # Conversion en format OpenCV (BGR)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        self.log({"event": "Image capturée depuis la Pi 5"}, log_name="default.json")

        return frame_bgr

    def get_servo_pwm(self, servo_num: int):
        """
        Récupère la valeur PWM brute (en microsecondes) d'un canal servo spécifique.

        Args:
            servo_num (int): Le numéro du servo (1 à 8) dont la valeur PWM doit être lue.
                             Le message SERVO_OUTPUT_RAW fournit 8 valeurs.

        Returns:
            int: La valeur PWM du servo en microsecondes (us).

        """
        if not 1 <= servo_num <= 8:
            raise ValueError(f"Le numéro de servo doit être entre 1 et 8. Reçu : {servo_num}")

        # Récupère le message SERVO_OUTPUT_RAW.
        msg = self.vehicle.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=3)

        if msg:
            # Les champs dans le message sont numérotés de servo1_raw à servo8_raw
            field_name = f'servo{servo_num}_raw'

            # Utilise getattr pour récupérer dynamiquement la valeur du bon champ
            pwm_value = getattr(msg, field_name)

            self.log({
                "event": "Valeur PWM servo obtenue.",
                "servo_id": servo_num,
                "pwm_us": pwm_value
            }, log_name="default.json")

            return pwm_value  # La valeur PWM est en microsecondes (us)

        if not msg:
            # Lève une erreur si le message n'a pas été reçu
            raise RuntimeError(f"Le message SERVO_OUTPUT_RAW est manquant ou indisponible après 3 secondes.")

    # Fonction pour démarrer le flux vidéo de la caméra rpi4
    def start_camera_rpi4(self, camera_index=0):
        """
        Initialise et démarre la caméra Pi 4 une seule fois.
        """
        if hasattr(self, "_camera4_started") and self._camera4_started:
            self.log({"event": "Caméra Pi 4 déjà démarrée"}, log_name="default.json")
            return

        if not self.camera_rpi4:
            cap = cv2.VideoCapture(camera_index)
            if not cap.isOpened():
                raise RuntimeError("Impossible d'ouvrir la caméra Pi 4")
            self.camera_rpi4 = cap
        else:
            # Si déjà définie, on s'assure qu'elle est bien ouverte
            if not self.camera_rpi4.isOpened():
                self.camera_rpi4.open(camera_index)

        self._camera4_started = True
        self.log({"event": "Caméra Pi 4 démarrée"}, log_name="default.json")

    # Fonction pour arrêter le flux vidéo de la caméra rpi4
    def stop_camera_rpi4(self):
        """
        Stoppe proprement la caméra Pi 4 et libère les ressources.
        """
        if hasattr(self, "_camera4_started") and self._camera4_started:
            try:
                if self.camera_rpi4 and self.camera_rpi4.isOpened():
                    self.camera_rpi4.release()
                self._camera4_started = False
                self.log({"event": "Caméra Pi 4 arrêtée"}, log_name="default.json")
            except Exception as e:
                self.log({"error": f"Erreur à l'arrêt de la caméra Pi 4 : {e}"}, log_name="default.json")
        else:
            self.log({"event": "Caméra Pi 4 déjà arrêtée ou non initialisée"}, log_name="default.json")

    # Fonction pour capturer une image du flux vidéo de la caméra rpi4
    def capture_camera_rpi4(self):
        """
        Capture une image depuis la caméra Pi 4 déjà démarrée.
        Ne stoppe pas la caméra après la capture.
        """
        if not hasattr(self, "_camera4_started") or not self._camera4_started:
            raise RuntimeError("La caméra Pi 4 n’est pas démarrée. Appeler start_camera_rpi4() d'abord.")

        if not self.camera_rpi4.isOpened():
            raise RuntimeError("Flux caméra Pi 4 non accessible")

        ret, frame = self.camera_rpi4.read()
        if not ret or frame is None:
            raise RuntimeError("Impossible de capturer une image avec la caméra Pi 4")

        self.log({"event": "Image capturée depuis la Pi 4"}, log_name="default.json")

        return frame

    def start_gps_thread(self):
        """
        Démarre un thread qui lit continuellement la position GPS
        et enregistre la dernière mise à jour reçue.
        """
        self.latest_lat = None
        self.latest_lon = None
        self.latest_alt = None
        self.last_gps_update = None
        self._gps_thread_running = True

        def gps_loop():
            while self._gps_thread_running:
                msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    self.latest_lat = msg.lat
                    self.latest_lon = msg.lon
                    self.latest_alt = msg.alt * 0.001
                    self.last_gps_update = time.time()
                time.sleep(0.05)

        self._gps_thread = threading.Thread(target=gps_loop, daemon=True)
        self._gps_thread.start()
        self.log({"event": "Thread GPS démarré"}, log_name="default.json")

    def stop_gps_thread(self):
        """
        Stoppe le thread GPS proprement.
        """
        self._gps_thread_running = False
        self.log({"event": "Thread GPS arrêté"}, log_name="default.json")

#----------------------------------------------
# Fonctions spécifiques à la mission 2
# ---------------------------------------------

    # Fonction qui donne la longueur d'un pixel en metre selon l'altitude (POUR FUTUNA !)
    def px_to_m(self, alt, image):
        height, _, _ = image.shape

        height_m = 0.9767 * alt
        px = height_m/height # 1px = ?m
        return px

    # Fonction qui renvoie la position gps d'un bbox
    def center_target_to_gps(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        factor = 0.8
        dx = factor*(cbx - cx)*px*-1 # *-1 car cameré à -180° par rapport à Futuna
        dy = factor*(cy - cby)*px*-1

        lat, lon, alt_global = self.get_pose()

        new_lat, new_lon, new_alt = pm.enu2geodetic(dx, dy, alt_global, lat/1e7, lon/1e7, alt_global)
        new_lat = round(new_lat, 6)
        new_lon = round(new_lon, 6)

        return new_lat, new_lon, new_alt

    # Fonction pour obtenir les seuils hsv pour une couleur cible
    def color_to_hsv(self, color):
        color = color.lower()
        if color == "red":
            lower_high = np.array([166, 70, 50])
            upper_high = np.array([180, 255, 255])
            lower_low = np.array([0, 85, 50])
            upper_low = np.array([5, 255, 255])
            return [lower_low, lower_high], [upper_low, upper_high]
        elif color == "orange":
            return [np.array([8, 85, 160])], [np.array([23, 255, 255])]
        elif color == "orange2":
            lower_high = np.array([166, 70, 50])
            upper_high = np.array([180, 255, 255])
            lower_low = np.array([0, 70, 50])
            upper_low = np.array([23, 255, 255])
            return [lower_low, lower_high], [upper_low, upper_high]
        elif color == "yellow":
            return [np.array([24, 80, 150])], [np.array([36, 255, 255])]
        elif color == "green":
            return [np.array([36, 85, 65])], [np.array([80, 255, 255])]
        elif color == "blue":
            return [np.array([95, 100, 140])], [np.array([110, 255, 255])]
        elif color == "purple":
            return [np.array([128, 85, 65])], [np.array([145, 255, 255])]
        elif color == "pink":
            return [np.array([145, 85, 65])], [np.array([169, 255, 255])]
        elif color == "brown":
            return [np.array([5, 80, 55])], [np.array([18, 220, 220])]
        elif color == "white":
            return [np.array([0, 0, 200])], [np.array([180, 30, 255])]
        elif color in ["grey", "gray"]:
            return [np.array([0, 5, 40])], [np.array([180, 30, 180])]
        elif color == "black":
            return [np.array([0, 0, 0])], [np.array([180, 255, 40])]
        else:
            raise ValueError(f"Color '{color}' is not defined")

    # Fonction pour filtrer une image selon plusieurs couleurs
    def filtre_image(self, image, colors, largeur=None):
        # Redimensionnement
        if largeur :
            (h, w) = image.shape[:2]
            ratio = largeur / float(w)
            hauteur = int(h * ratio)
            image = cv2.resize(image, (largeur, hauteur), interpolation=cv2.INTER_AREA)

        # Conversion en HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Image blanche de base
        white_img = np.ones_like(image, dtype=np.uint8) * 255

        # Masque global (tout ce qu'on veut garder)
        global_mask = None

        for color in colors:
            lowers, uppers = self.color_to_hsv(color)

            # Gestion spéciale si plusieurs intervalles (cas du rouge par ex.)
            for lower, upper in zip(lowers, uppers):
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.dilate(mask, None, iterations=3)  # fusion zones proches

                if global_mask is None:
                    global_mask = mask
                else:
                    global_mask = cv2.bitwise_or(global_mask, mask)

        # Masque inverse (zones à mettre en blanc)
        mask_white = cv2.bitwise_not(global_mask)

        # Partie filtrée
        seg_img_colors = cv2.bitwise_and(image, image, mask=global_mask)

        # Partie en blanc
        seg_img_white = cv2.bitwise_and(white_img, white_img, mask=mask_white)

        # Fusion finale
        result = cv2.add(seg_img_colors, seg_img_white)

        return result

    # Fonction qui renvoie le bbox de la target
    def center_target(self, image, area_threshold=0):
        # Convertir en gris et créer un masque des zones colorées
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Tout ce qui n'est pas blanc devient 1
        mask = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY_INV)[1]

        # Trouver les contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Chercher le contour avec la plus grande aire
        max_contour = max(contours, key=cv2.contourArea)
        max_area = cv2.contourArea(max_contour)

        if max_area < area_threshold:
            return None

        # Bounding box
        x, y, w, h = cv2.boundingRect(max_contour)
        cx = x + w // 2
        cy = y + h // 2

        return cx, cy

    # Fonction pour déplacer le drone sur la cible avec des mouvements relatifs
    def goto_relative(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        dx = (cx - cbx)*px # *-1 car cameré à -180° par rapport à Futuna
        dy = (cy - cby)*px

        # Seuil de tolérance
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            self.log({"event":"Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            return

        # Vitesse fixe (1 m/s)
        speed = 1.0

        # Durées proportionnelles aux distances
        duration_x = abs(dy) / speed
        duration_y = abs(dx) / speed

        # Directions de déplacement
        vy = speed if dx > 0 else -speed if dx < 0 else 0
        vx = -speed if dy > 0 else speed if dy < 0 else 0

        self.log({"event":"Déplacement relatif vers la cible.", "dx":dx, "dy":dy, 
                  "duration_x":duration_x, "duration_y":duration_y, "vx":vx, "vy":vy}, log_name="default.json")
        
        factor = 1
        # Mouvement sur X
        if abs(dy) >= 0.1:
            self.log({"event":"Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0, duration_x*factor)

        time.sleep(1)

        # Mouvement sur Y
        if abs(dx) >= 0.1:
            self.log({"event":"Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0, duration_y*factor)

    # Fonction pour déplacer le drone sur la cible avec des mouvements relatifs et PID
    def goto_relative_p(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        dx = (cx - cbx) * px  # *-1 car caméra à -180° par rapport à Futuna
        dy = (cy - cby) * px

        # Seuil de tolérance
        if abs(dx) < 0.5 and abs(dy) < 0.5:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 0.25  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy * -1 # inversion car x/y du drone et de l'image sont inversés
        vy_pid = kp * dx

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        max_speed = 4.0  # m/s
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        # Durées proportionnelles aux distances (comme avant)
        duration_x = vx / abs(dy)
        duration_y = vy / abs(dx)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": duration_x, "duration_y": duration_y,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 1 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if abs(dy) >= 1:
            self.log({"event": "Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0, duration_x * factor)

        time.sleep(1)

        # Mouvement sur Y
        if abs(dx) >= 1:
            self.log({"event": "Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0, duration_y * factor)
 
    # Fonction pour déplacer le drone sur la cible avec des mouvements relatifs et PID
    def tracking_fast(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        dx = (cx - cbx) * px  # *-1 car caméra à -180° par rapport à Futuna
        dy = (cy - cby) * px

        # Seuil de tolérance
        if abs(dx) < 0.5 and abs(dy) < 0.5:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 1  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy * -1 # inversion car x/y du drone et de l'image sont inversés + image à 180°
        vy_pid = kp * dx

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        max_speed = 4.0  # m/s
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": 0.1, "duration_y": 0.1,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 1 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if abs(dy) >= 1:
            self.log({"event": "Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0.1, 0.1 * factor)

        time.sleep(0.1)

        # Mouvement sur Y
        if abs(dx) >= 1:
            self.log({"event": "Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0.1, 0.1 * factor)

    # Fonction pour déplacer le drone sur la cible avec des mouvements relatifs et PID
    def tracking_slow(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        dx = (cx - cbx) * px  # *-1 car caméra à -180° par rapport à Futuna
        dy = (cy - cby) * px

        # Seuil de tolérance
        if alt>20:
            seuil = 1.2
            max_speed = 0.6  # m/s
        elif alt>18:
            seuil = 1
            max_speed = 0.6  # m/s
        elif alt>14:
            seuil = 0.8
            max_speed = 0.6  # m/s
        elif alt>10:
            seuil = 0.5
            max_speed = 0.6  # m/s
        elif alt>6:
            seuil = 0.6
            max_speed = 0.3  # m/s
        else:
            seuil = 0.7
            max_speed = 0.1  # m/s

        if abs(dx) < seuil and abs(dy) < seuil:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            self.move_down(0.5, 0.1)
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 1  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy * -1 # inversion car x/y du drone et de l'image sont inversés + image à 180°
        vy_pid = kp * dx

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": 0.1, "duration_y": 0.1,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 1 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if abs(dy) >= 1:
            self.log({"event": "Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0, 0.1 * factor)

        time.sleep(0.1)

        # Mouvement sur Y
        if abs(dx) >= 1:
            self.log({"event": "Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0, 0.1 * factor)
                 
    # Fonction pour détecter les personnes dans une image avec YOLO
    def find_targets(self, image):
        # Charger le modèle (par exemple yolo11x.pt)
        model = YOLO("Modeles/yolo11x.pt")

        # Effectuer la détection
        results = model(image)[0]

        people = []

        # Récupérer les boxes des personnes détectées
        for box in results.boxes:
            cls = int(box.cls[0])
            if model.names[cls] == "person":
                people.append(box)
        
        return people

    # Fonction pour détecter une personne dans une image avec YOLO
    def find_target(self, image):
        # Charger le modèle (par exemple yolo11x.pt)
        model = YOLO("Modeles/yolo11x.pt")

        # Effectuer la détection
        results = model(image)[0]

        # Récupérer les boxes des personnes détectées
        for box in results.boxes:
            cls = int(box.cls[0])
            if model.names[cls] == "person":
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                return cx, cy
        return None

    # Fonction pour choisir la personne la plus proche des couleurs cibles
    def choose_target(self, image, people, colors):
        if not people or not colors:
            return None

        # On récupère une image filtrée : seules les zones des couleurs demandées sont visibles
        filtered_img = self.filtre_image(image, colors)

        # Conversion en masque binaire : pixels colorés = 1, le reste = 0
        mask = cv2.inRange(filtered_img, np.array([0, 0, 0]), np.array([254, 254, 254]))
        mask = cv2.bitwise_not(mask)  # inverser (on veut les zones colorées)

        scores = []
        for box in people:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            roi_mask = mask[y1:y2, x1:x2]
            if roi_mask.size == 0:
                scores.append(0)
                continue

            # Pourcentage de pixels "colorés" dans la bbox
            score = np.sum(roi_mask > 0) / roi_mask.size
            scores.append(score)

        if not scores:
            return None

        best_idx = int(np.argmax(scores))
        best_score = scores[best_idx]

        if best_score < 0.01:
            return None
        
        best_box = people[best_idx]
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        return cx, cy

    # Fonction d'asservissement de thomas      
    def asservissement_thomas(self, image, aruco_center_x, aruco_center_y, alt_stab = None):
        height, width, _ = image.shape
        x_imageCenter = int(width / 2)
        y_imageCenter = int(height / 2)

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction (on monte)
        if aruco_center_x == None:
            self.set_velocity(0, 0, -0.2) #sens z positif -> vers le sol

        # Récupération de l'altitude du drone
        altitude = self.get_alt_lidar()  
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        self.kp_atterrissage = 0.005 if altitude < 5 else 0.008
        self.kp_atterrissage *= self.coefficient_kp_atterrissage

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = x_imageCenter - aruco_center_x
        erreurY = y_imageCenter - aruco_center_y
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.get_yaw()
        print("alpha = " + str(alpha))
        erreurEst = dist_center * cos(alpha)
        erreurNord = dist_center * sin(alpha)

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeEst = (erreurEst - self.erreurAnterieureEst_atterrissage)
        erreurDeriveeNord = (erreurNord - self.erreurAnterieureNord_atterrissage)
        # Erreur intégrale
        self.erreurIntegraleEst_atterrissage += erreurEst
        self.erreurIntegraleNord_atterrissage += erreurNord
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureEst_atterrissage = erreurEst
        self.erreurAnterieureNord_atterrissage = erreurNord

        # Calcul de la vitesse corrigée 
        vy = self.kp_atterrissage * erreurEst + self.kd_atterrissage * erreurDeriveeEst + self.ki_atterrissage * self.erreurIntegraleEst_atterrissage
        vx = self.kp_atterrissage * erreurNord + self.kd_atterrissage * erreurDeriveeNord + self.ki_atterrissage * self.erreurIntegraleNord_atterrissage        
        # Bornage des vitesses à +/- 5 m/s
        vy = min(max(vy, -5.0), 5.0)
        vx = min(max(vx, -5.0), 5.0)
        
        if alt_stab is None :
            # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus de l'aruco 
            dist_center_threshold = 50 if altitude < 2 else 1000        
            # Si n'est dans un rayon d'un mètre autour du drone, il ne change pas son altitude 
            if dist_center > dist_center_threshold :
                vz = 0
            # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
            else:
            #Choix de la vitesse verticale en fonction de l'altitude
                if altitude > 10:
                    vz = 1.5 
                elif altitude > 5:
                    vz = 0.5
                else:
                    vz = 0.2
        else :
            # On definie la distance a partir de laquelle on considère le drone au dessus de la piscine
            # Comme la piscine fait 3 x 3 m, on peut considéré etre au dessus lorsqu'on se trouve a moins de 1 m de son centre
            #  
            dist_center_threshold = 50 if altitude < 7 else 100          
            if dist_center > dist_center_threshold :
                vz = 0
            # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
            else:
            #Choix de la vitesse verticale en fonction de l'altitude
                if altitude > 10:
                    vz = 1.5 
                elif altitude > 5:
                    vz = 0.5
                else:
                    vz = 0.2
                
                #vz = (altitude - alt_stab) * self.kp_hauteur
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : vy = " + str(vy) + " ; vx = " + str(vx) + " ; VZ = " + str(vz))
        self.set_velocity(vx, vy, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'

    # Fonction tracking fonctionnelle
    def tracking(self, alt, image, cbx, cby):
        height, width, _ = image.shape
        cx = int(width / 2)
        cy = int(height / 2)

        px = self.px_to_m(alt, image)
        dx = (cx - cbx) * px  # *-1 car caméra à -180° par rapport à Futuna
        dy = (cy - cby) * px

        # Seuil de tolérance
        if alt>20:
            seuil = 1.6
            max_speed = 0.55  # m/s
        elif alt>18:
            seuil = 1.3
            max_speed = 0.5  # m/s
        elif alt>14:
            seuil = 1
            max_speed = 0.45  # m/s
        elif alt>10:
            seuil = 0.7
            max_speed = 0.4  # m/s
        elif alt>7:
            seuil = 0.6
            max_speed = 0.3  # m/s
        else:
            seuil = 0.7
            max_speed = 0.1  # m/s

        if abs(dx) < seuil and abs(dy) < seuil:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            self.move_down(0.5, 0.1)
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 1  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy * -1 # inversion car x/y du drone et de l'image sont inversés + image à 180°
        vy_pid = kp * dx

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": 0.1, "duration_y": 0.1,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 1 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if abs(dy) >= 1:
            self.log({"event": "Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0, 0.1 * factor)

        time.sleep(0.1)

        # Mouvement sur Y
        if abs(dx) >= 1:
            self.log({"event": "Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0, 0.1 * factor)
                 
#----------------------------------------------
# Fonctions spécifiques à la mission 3
# ---------------------------------------------

    # Fonction pour calculer une nouvelle position GPS à partir d'une position de départ
    def dist_to_new_pos(lat, lon, alt, dist_m, angle=0, rad=False):
        """
        Calcul d'une nouvelle position à partir d'une position de départ, 
        d'une distance et d'un angle (par rapport au nord)
        """
        if not rad:
            while angle < 0:
                angle += 360
            angle = angle % 360
            angle = math.radians(angle)
        dist_n = dist_m * math.cos(angle)
        dist_e = dist_m * math.sin(angle)
        R = 6371000  # Rayon de la Terre en mètres
        new_lat = lat + (dist_n / R) * (180 / math.pi)
        new_lon = lon + (dist_e / R) * (180 / math.pi) / math.cos(lat * math.pi / 180)
        return Position(new_lat, new_lon, alt)

    # Fonction pour calculer la distance nord (x), est (y) et verticale (z) entre deux positions GPS
    @staticmethod
    def pos_to_dist(lat1, lon1, alt1, lat2, lon2, alt2):
        """
        Calcule la distance nord (x), est (y) et verticale (z) en mètres entre deux positions GPS.
        Utilise une approximation locale (petites distances).
        """
        R = 6371000  # Rayon de la Terre en mètres
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        avg_lat = math.radians((lat1 + lat2) / 2)

        dx = d_lat * R  # distance nord (m)
        dy = d_lon * R * math.cos(avg_lat)  # distance est (m)
        dz = alt2 - alt1  # distance verticale (m)

        return dx, dy, dz
    
    # Fonction pour demander un flux de données GLOBAL_POSITION_INT à 100ms (10 Hz)
    def request_global_position_stream(self, rate_hz=10):
        """
        Demande le flux de données GLOBAL_POSITION_INT à la fréquence spécifiée
        rate_hz: Fréquence en Hz (par défaut 10 Hz = 100ms)
        """
        if not self.vehicle:
            print("Erreur: Drone non connecté")
            return

        # Méthode 1: Utiliser SET_MESSAGE_INTERVAL (recommandé pour ArduPilot récent)
        interval_us = int(1000000 / rate_hz)  # Conversion Hz vers microsecondes
        
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # param1: message ID
            interval_us,  # param2: interval en microsecondes
            0, 0, 0, 0, 0  # param3-7: non utilisés
        )
        self.log({"event": f"Flux GLOBAL_POSITION_INT demandé à {rate_hz} Hz"}, log_name="default.json")

    def start_uwb_serial(self, port="/dev/ttyUSB0", baud=115200):
        """Initialise la connexion série vers le module UWB."""
        try:
            self.uwb_serial = serial.Serial(port, baud, timeout=1)
            self.log({"event": "[UWB] Connexion au module réussie"}, log_name="default.json")
        except Exception as e:
            self.uwb_serial = None
            print(f"Erreur de connexion au module UWB: {e}")
            self.log({"event": "[UWB] ERREUR connexion module"}, log_name="default.json")
            
            
    def request_uwb_measure(self):
        """
        Envoie '1' et lit la réponse si disponible.
        Retourne None si rien reçu.
        """
        if self.uwb_serial is None:
            self.log({"event": "[UWB] Port non initialisé"}, log_name="default.json")
            return None

        try:
            self.uwb_serial.write(b"1")
        except Exception as e:
            self.log({"event": "[UWB] ERREUR écriture"}, log_name="default.json")
            return None

        if self.uwb_serial.in_waiting > 0:
            data = self.uwb_serial.readline().decode("utf-8", errors="ignore").strip()
            if data:
                return data

        return None
    
    def request_uwb_autocalibration(self, distance, timeout=15):
        """
        Envoie d'abord '2' puis la distance séparément pour lancer l'autocalibration.
        La distance doit être comprise entre 1 et 10 (en mètres).
        Attend une réponse du port série avant de continuer.
        Retourne la ligne reçue ou None si timeout.
        """

        
        try:
            d = int(distance)
        except ValueError:
            self.log({"event": "[UWB] Distance invalide (non numérique)"}, log_name="default.json")
            return None

        if d < 1 or d > 10:
            self.log({"event": "[UWB] Erreur distance hors limites (1–10)"}, log_name="default.json")
            return None

        if self.uwb_serial is None:
            self.log({"event": "[UWB] Port non initialisé"}, log_name="default.json")
            return None

    
        try:
            
            self.uwb_serial.write(b"2")# commande autocalibration

            self.uwb_serial.write(bytes(d))# envoi de la distance
        except Exception as e:
            self.log({"event": "[UWB] ERREUR écriture", "error": str(e)}, log_name="default.json")
            return None
    
        start = time.time()

        while time.time() - start < timeout:
            if self.uwb_serial.in_waiting > 0:
                try:
                    data = self.uwb_serial.readline().decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue

                if data:
                    self.log({"event": "[UWB] Réponse autocalibration", "data": data},
                            log_name="default.json")
                    return data

            time.sleep(0.01)

        # Timeout
        self.log({"event": "[UWB] Timeout autocalibration"}, log_name="default.json")
        return None
    
    def who_am_i_uwb(self):
        """
        Envoie '3' et lit la réponse du module UWB.
        Retourne 'TAG', 'ANCRE' ou None.
        """
        if self.uwb_serial is None:
            self.log({"event": "[UWB] Port non initialisé"}, log_name="default.json")
            return None

        try:
            self.uwb_serial.write(b"3")
        except Exception as e:
            self.log({"event": "[UWB] ERREUR écriture"}, log_name="default.json")
            return None

        if self.uwb_serial.in_waiting > 0:
            data = self.uwb_serial.readline().decode("utf-8", errors="ignore").strip()

            if data == "0":
                return "TAG"
            elif data == "1":
                return "ANCRE"
            else:
                # Réponse inattendue
                self.log({"event": f"[UWB] Réponse inconnue: {data}"}, log_name="default.json")
                return None

        return None



#----------------------------------------------
# Fonctions spécifiques à la mission 4
# ---------------------------------------------

#----------------------------------------------
# Fonctions spécifiques au projet IOT caméra IR
# ---------------------------------------------

    def filtre_image_IR(self, image, seuil=0.8):
        if image.dtype != np.uint8:
            raise ValueError("L'image doit être en uint8.")

        # Normalisation 0–1
        img_norm = image.astype(np.float32) / 255.0

        # Si l'image a 3 canaux, on passe en intensité max
        if img_norm.ndim == 3:
            img_norm = img_norm.max(axis=2)

        # Binarisation
        img_bin = (img_norm >= seuil).astype(np.float32)
        print(f"Filtrage IR : min={img_bin.min()}, max={img_bin.max()}")

        return img_bin  # 0 ou 1

    def center_target_IR(self, img_bin, area_threshold=0):
        if img_bin.dtype != np.uint8:
            img_bin = (img_bin * 255).astype(np.uint8)

        # Trouver les contours
        contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Plus grand contour
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area < area_threshold:
            return None

        # Bounding box
        x, y, w, h = cv2.boundingRect(c)
        cx = x + w // 2
        cy = y + h // 2

        return cx, cy

    def tracking_IR(self, alt, image, cbx, cby):
        shape = image.shape
        height = shape[0]
        width = shape[1]
        cx = int(width / 2)
        cy = int(height / 2)

        dx = (cx - cbx)
        dy = (cy - cby)

        # Seuil de tolérance
        if alt>20:
            seuil = 1.6
            max_speed = 0.55  # m/s
        elif alt>18:
            seuil = 1.3
            max_speed = 0.5  # m/s
        elif alt>14:
            seuil = 1
            max_speed = 0.85  # m/s
        elif alt>10:
            seuil = 0.7
            max_speed = 0.8  # m/s
        elif alt>7:
            seuil = 0.6
            max_speed = 0.6  # m/s
        else:
            seuil = 0.7
            max_speed = 0.1  # m/s

        if abs(dx) < seuil and abs(dy) < seuil:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 1  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy 
        vy_pid = kp * dx * -1

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": 0.1, "duration_y": 0.1,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 10 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if abs(dy) >= 1:
            self.log({"event": "Mouvement sur X."}, log_name="default.json")
            self.move_velocity(vx, 0, 0, 0.1 * factor)

        time.sleep(0.1)

        # Mouvement sur Y
        if abs(dx) >= 1:
            self.log({"event": "Mouvement sur Y."}, log_name="default.json")
            self.move_velocity(0, vy, 0, 0.1 * factor)

    def tracking_IR_fast(self, alt, image, cbx, cby):
        shape = image.shape
        height = shape[0]
        width = shape[1]
        cx = int(width / 2)
        cy = int(height / 2)

        dx = (cx - cbx)
        dy = (cy - cby)

        # Seuil de tolérance
        if alt>20:
            seuil = 1.6
            max_speed = 0.55  # m/s
        elif alt>18:
            seuil = 1.3
            max_speed = 0.5  # m/s
        elif alt>14:
            seuil = 1
            max_speed = 0.85  # m/s
        elif alt>10:
            seuil = 0.7
            max_speed = 0.8  # m/s
        elif alt>7:
            seuil = 0.6
            max_speed = 0.6  # m/s
        else:
            seuil = 0.7
            max_speed = 0.1  # m/s

        if abs(dx) < seuil and abs(dy) < seuil:
            self.log({"event": "Cible déjà centrée, pas de déplacement."}, log_name="default.json")
            return

        # --- Paramètres PID (proportionnel uniquement pour l'instant) ---
        kp = 1  # ajustable selon la réactivité souhaitée
        vx_pid = kp * dy 
        vy_pid = kp * dx * -1

        # Bornage des vitesses pour éviter d'envoyer des valeurs trop grandes
        vx = max(min(vx_pid, max_speed), -max_speed)
        vy = max(min(vy_pid, max_speed), -max_speed)

        self.log({
            "event": "Déplacement relatif vers la cible avec P",
            "dx": dx, "dy": dy,
            "duration_x": 0.1, "duration_y": 0.1,
            "vx": vx, "vy": vy,
            "kp": kp
        }, log_name="default.json")

        factor = 10 # facteur à modifier selon la précision du drone
        # Mouvement sur X
        if (abs(dy) >= 1 or abs(dx) >= 1):
            print(dy, dx)
            self.log({"event": "Mouvement"}, log_name="default.json")
            self.move_velocity(vx, vy, 0, 0.1 * factor)
            
    def hymne(self):
        print("La Marseillaise, c'est parti !")

        time.sleep(0.1)
        self.bip_melodie("T80L1")
        time.sleep(1)

        self.bip_melodie("T120L1D6")
        time.sleep(0.125)
        self.bip_melodie("T100L1D6")
        time.sleep(0.375)
        self.bip_melodie("T120L1D6")
        time.sleep(0.125)
        self.bip_melodie("T90L1G6")
        time.sleep(0.5)
        self.bip_melodie("T90L1G6")
        time.sleep(0.5)
        self.bip_melodie("T90L1A6")
        time.sleep(0.5)
        self.bip_melodie("T90L1A6")
        time.sleep(0.5)
        self.bip_melodie("T80L1B6")
        time.sleep(0.75)
        self.bip_melodie("T110L1B6")
        time.sleep(0.25)
        self.bip_melodie("T100L1G6")
        time.sleep(0.375)
        self.bip_melodie("T120L1G6")
        time.sleep(0.125)
        self.bip_melodie("T100L1B6")
        time.sleep(0.375)
        self.bip_melodie("T120L1G6")
        time.sleep(0.125)
        self.bip_melodie("T90L1E6")
        time.sleep(0.5)
        self.bip_melodie("T70L1B6")
        time.sleep(1)
        self.bip_melodie("T100L1A6")
        time.sleep(0.375)
        self.bip_melodie("T120L1F6")
        time.sleep(0.125)
        self.bip_melodie("T70L1G6")
        time.sleep(1)

        self.bip_melodie("T120L1")

        print("Fin de l'hymne...")