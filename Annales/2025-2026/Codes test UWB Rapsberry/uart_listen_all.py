import serial
import time

# ouverture port série baud = 115200
ser = serial.Serial('/dev/serial0', 115200, timeout =1)

print("lecture UART en cours")

try:
    while True:
        # ne fait que écouter
        if ser.in_waiting > 0:
            data =ser.readline().decode('utf-8',errors='ignore').strip()
            if data:
                print(f"Message reçu : {data}")
        time.sleep(0.01)

# attend intervention clavier pour stopper
except KeyboardInterrupt:
    print("\nArrêt du programme")
    ser.close()