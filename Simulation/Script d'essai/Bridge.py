import serial, socket, threading, time

COM = "COM6"                        # port de l'ESP32 lui-même
BAUD = 115200
SITL = ("172.27.138.121", 14552)    # IP de WSL (hostname -I) : change après un redémarrage
SNIFF = ("127.0.0.1", 14590)        # ancien 14560 = port de Mission Planner (drone 2) !
DEBUG = True                        # affiche chaque trame décodée

MSG_NAMES = {0: "HEARTBEAT", 11: "SET_MODE", 20: "PARAM_REQUEST_READ",
             22: "PARAM_VALUE", 24: "GPS_RAW_INT", 30: "ATTITUDE",
             33: "GLOBAL_POSITION_INT", 76: "COMMAND_LONG", 77: "COMMAND_ACK",
             83: "ATTITUDE_TARGET", 253: "STATUSTEXT"}

# Ouverture sans lever DTR/RTS (évite de resetter l'ESP32 à chaque lancement)
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = COM, BAUD, 0.01
ser.dtr = False
ser.rts = False
ser.open()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 0))
try:
    sock.ioctl(socket.SIO_UDP_CONNRESET, False)
except (AttributeError, OSError):
    pass

n_up = n_down = 0
n_hb = n_cmd = n_text = 0           # heartbeats / autres trames MAVLink / octets de texte
buf = bytearray()


def analyse(data):
    """Décode le flux ESP32 : trames MAVLink 1/2 ou texte parasite."""
    global buf, n_hb, n_cmd, n_text
    buf += data
    if len(buf) > 600:              # flux illisible : on repart de zéro
        buf.clear()
        return
    while buf:
        if buf[0] == 0xFD:          # MAVLink 2
            if len(buf) < 12:
                return
            total = 12 + buf[1] + (13 if buf[2] & 1 else 0)
            if len(buf) < total:
                return
            f = bytes(buf[:total]); del buf[:total]
            sysid, msgid = f[5], f[7] | f[8] << 8 | f[9] << 16
        elif buf[0] == 0xFE:        # MAVLink 1
            if len(buf) < 8:
                return
            total = 8 + buf[1]
            if len(buf) < total:
                return
            f = bytes(buf[:total]); del buf[:total]
            sysid, msgid = f[3], f[5]
        else:                       # texte : jusqu'au prochain début de trame
            i = next((k for k, b in enumerate(buf) if b in (0xFD, 0xFE)), len(buf))
            txt = bytes(buf[:i]); del buf[:i]
            n_text += len(txt)
            if DEBUG:
                print("  TEXTE (pas du MAVLink) :", txt)
            continue
        if msgid == 0:
            n_hb += 1
        else:
            n_cmd += 1
        if DEBUG:
            print(f"  MAVLink sysid={sysid} msgid={msgid} "
                  f"({MSG_NAMES.get(msgid, '?')}) {total} octets")


def serial_to_udp():
    global n_up
    while True:
        data = ser.read(256)
        if data:
            n_up += len(data)
            analyse(data)
            for dst in (SITL, SNIFF):
                try: sock.sendto(data, dst)
                except OSError: pass


def stats():
    while True:
        time.sleep(2)
        print(f"ESP32->SITL: {n_up} o | SITL->ESP32: {n_down} o | "
              f"heartbeats={n_hb} autres_trames={n_cmd} texte={n_text} o")


threading.Thread(target=serial_to_udp, daemon=True).start()
threading.Thread(target=stats, daemon=True).start()
print(f"Pont : {COM}@{BAUD} -> {SITL[0]}:{SITL[1]}")

while True:
    try:
        data, _ = sock.recvfrom(2048)
        n_down += len(data)
        ser.write(data)
    except ConnectionResetError:
        continue