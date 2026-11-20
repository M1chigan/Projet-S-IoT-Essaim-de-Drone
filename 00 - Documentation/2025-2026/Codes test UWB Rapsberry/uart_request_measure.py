import serial
import time

# ouverture port série baud = 115200
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout =1)

try:
    while True:
        # 1 = 49 en ASCII
        message = "1"
        ser.write(message.encode())
        print("msg sent")
        # attente réponse
        if ser.in_waiting > 0:
            data =ser.readline().decode('utf-8',errors='ignore').strip()
            if data:
                print(f"Message reçu : {data}")
        
        time.sleep(2)

# attend intervention clavier pour stopper
except KeyboardInterrupt:
    print("\nArrêt du programme")
    ser.close()
