"""Pont : port COM de l'ESP32  <->  port TCP du simulateur ArduPilot.

Installation :  pip install pyserial
Utilisation  :  python bridge.py COM5            (TCP 5762 par défaut)
                python bridge.py COM5 5763       (autre port TCP)
"""
import socket
import sys
import threading

import serial

com_port = "COM6"   # remplacez par le port de votre ESP32
tcp_port = 5762    # essayez 5763 si 5762 refuse la connexion2

ser = serial.Serial(com_port, 115200, timeout=0.01)
sock = socket.create_connection(("127.0.0.1", tcp_port))
print(f"Pont actif : {com_port} <-> 127.0.0.1:{tcp_port}")

n_to_sim = 0
n_to_esp = 0


def esp_to_sim():
    global n_to_sim
    while True:
        data = ser.read(256)
        if data:
            sock.sendall(data)
            n_to_sim += len(data)


def sim_to_esp():
    global n_to_esp
    while True:
        data = sock.recv(256)
        if not data:
            print("Connexion au simulateur fermée.")
            break
        ser.write(data)
        n_to_esp += len(data)


threading.Thread(target=esp_to_sim, daemon=True).start()
t = threading.Thread(target=sim_to_esp, daemon=True)
t.start()

try:
    while t.is_alive():
        t.join(2)
        print(f"ESP32 -> simulateur : {n_to_sim} octets | simulateur -> ESP32 : {n_to_esp} octets")
except KeyboardInterrupt:
    pass