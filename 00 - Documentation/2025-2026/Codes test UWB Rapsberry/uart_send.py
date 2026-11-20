import serial
import time

# ouverture port série baud = 115200
ser = serial.Serial('/dev/serial0', 115200, timeout =1)

try:
    while True:
        # envoi message de test
        message = "test\n"
        ser.write(message.encode())
        print("msg sent")
        time.sleep(1)

# attend intervention clavier pour stopper
except KeyboardInterrupt:
    print("\nArrêt du programme")
    ser.close()
