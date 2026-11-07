import serial, socket, threading, time

COM = "COM6"           # port de l'ESP32 lui-même
BAUD = 115200
SITL = ("172.27.138.121", 14552)
SNIFF = ("127.0.0.1", 14560)

ser = serial.Serial(COM, BAUD, timeout=0.01)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 0))
try:
    sock.ioctl(socket.SIO_UDP_CONNRESET, False)
except (AttributeError, OSError):
    pass

n_up = n_down = 0

def serial_to_udp():
    global n_up
    while True:
        data = ser.read(256)
        if data:
            n_up += len(data)
            for dst in (SITL, SNIFF):
                try: sock.sendto(data, dst)
                except OSError: pass

def stats():
    while True:
        time.sleep(2)
        print(f"ESP32->SITL: {n_up} octets | SITL->ESP32: {n_down} octets")

threading.Thread(target=serial_to_udp, daemon=True).start()
threading.Thread(target=stats, daemon=True).start()

while True:
    try:
        data, _ = sock.recvfrom(2048)
        n_down += len(data)
        ser.write(data)
    except ConnectionResetError:
        continue