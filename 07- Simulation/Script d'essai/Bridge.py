"""
bridge_uwb.py (Windows) : ESP32 <-> drone SITL (TCP direct) + simulation UWB.

  - Relaie MAVLink entre l'ESP32 (COM) et SERIAL1 du drone ME (5762, 5772, 5782).
  - Lit les positions des N drones sur leur SERIAL2 (5763, 5773, 5783).
  - Calcule la distance bruitee entre ME et chaque autre drone et l'ecrit dans
    le flux de l'ESP32 : NAMED_VALUE_FLOAT "d<id>" (sysid 250), a RATE_HZ.
  - Recupere les mesures filtrees renvoyees par l'ESP32 ("f<id>" accepte,
    "x<id>" rejete par la porte) et journalise tout dans uwb_log.csv.

Usage :  python bridge_uwb.py [COM] [ME]      ex : python bridge_uwb.py COM6 1
Prerequis : pip install pyserial pymavlink
"""
import csv, math, random, serial, socket, sys, threading, time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

COM = sys.argv[1] if len(sys.argv) > 1 else "COM6"
ME = int(sys.argv[2]) if len(sys.argv) > 2 else 1     # SYSID du drone de cet ESP32
N_DRONES = 3
HOST = "127.0.0.1"                                    # sinon : IP de WSL
BAUD = 115200                                         # = PIXHAWK_BAUDRATE
ESP_PORT = 5762 + 10 * (ME - 1)                       # SERIAL1 du drone ME

SIM_SYSID = 250                                       # = SIM_SYSID dans config.h
RATE_HZ = 10                                          # mesures UWB par seconde et par drone
SIGMA = 0.10                                          # bruit (m)
OUTLIER_P = 0.02                                      # proba de mesure aberrante
OUTLIER_M = 1.5                                       # amplitude de l'aberration (m)
M_PER_DEG = 111319.5

# --- Port serie (sans lever DTR/RTS) ---
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = COM, BAUD, 0.01
ser.dtr = False
ser.rts = False
ser.open()
wlock = threading.Lock()


def write_serial(b):
    with wlock:
        ser.write(b)


# --- Journal CSV ---
t0 = time.time()
llock = threading.Lock()
log_f = open("uwb_log.csv", "w", newline="")
log = csv.writer(log_f)
log.writerow(["t_s", "kind", "id", "value"])    # t=vrai n=bruite f=filtre x=rejete


def log_row(kind, ident, v):
    with llock:
        log.writerow([f"{time.time() - t0:.3f}", kind, ident, f"{v:.4f}"])


# --- Lecture des rapports de l'ESP32 ---
parser = mavlink2.MAVLink(None)
parser.robust_parsing = True


def handle_esp32(data):
    for m in parser.parse_buffer(data) or []:
        if m.get_type() != "NAMED_VALUE_FLOAT":
            continue
        name = m.name if isinstance(m.name, str) else m.name.decode(errors="ignore")
        name = name.rstrip("\x00")
        if len(name) >= 2 and name[0] in "fx" and name[1:].isdigit():
            log_row(name[0], int(name[1:]), m.value)


# --- Liaison TCP avec le drone ---
sock = None
n_up = n_down = n_uwb = 0
last = {}


def connect():
    global sock
    while True:
        try:
            s = socket.create_connection((HOST, ESP_PORT), timeout=3)
            s.settimeout(0.01)
            sock = s
            print(f"Connecte a SITL {HOST}:{ESP_PORT} (drone {ME})")
            return
        except OSError:
            print(f"SITL injoignable sur {HOST}:{ESP_PORT}, nouvel essai dans 2 s...")
            time.sleep(2)


def drop():
    global sock
    if sock:
        try: sock.close()
        except OSError: pass
    sock = None


def serial_to_tcp():
    global n_up
    while True:
        data = ser.read(256)
        if data:
            handle_esp32(data)
            if sock:
                try:
                    sock.sendall(data)
                    n_up += len(data)
                except OSError:
                    drop()


# --- Simulation UWB ---
def feeder():
    global n_uwb
    links = {}
    for s in range(1, N_DRONES + 1):
        while True:
            try:
                links[s] = mavutil.mavlink_connection(
                    f"tcp:{HOST}:{5763 + 10 * (s - 1)}",
                    source_system=SIM_SYSID, source_component=190)
                break
            except OSError:
                print(f"SERIAL2 du drone {s} injoignable, nouvel essai...")
                time.sleep(2)
    for s, l in links.items():
        l.wait_heartbeat(timeout=10)
        for msgid in (164, 33):                       # SIMSTATE, GLOBAL_POSITION_INT
            l.mav.command_long_send(s, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                    0, msgid, int(1e6 / RATE_HZ), 0, 0, 0, 0, 0)
    print("Simulation UWB : positions lues sur", sorted(links))

    latlon, alt, use_sim = {}, {}, set()
    origin = None
    out = mavlink2.MAVLink(None, SIM_SYSID, 191)

    def xyz(s):
        n = (latlon[s][0] - origin[0]) * M_PER_DEG
        e = (latlon[s][1] - origin[1]) * M_PER_DEG * math.cos(math.radians(origin[0]))
        return n, e, alt.get(s, 0.0)

    nxt = time.time()
    while True:
        for s, l in links.items():
            while True:
                m = l.recv_match(blocking=False)
                if m is None:
                    break
                t = m.get_type()
                if t == "SIMSTATE":                   # verite terrain
                    latlon[s] = (m.lat / 1e7, m.lng / 1e7)
                    use_sim.add(s)
                elif t == "GLOBAL_POSITION_INT":
                    alt[s] = m.alt / 1000.0
                    if s not in use_sim:
                        latlon[s] = (m.lat / 1e7, m.lon / 1e7)
                if origin is None and 1 in latlon:
                    origin = latlon[1]

        now = time.time()
        if now >= nxt and origin and ME in latlon:
            nxt = now + 1.0 / RATE_HZ
            for s in links:
                if s == ME or s not in latlon:
                    continue
                d_true = math.dist(xyz(ME), xyz(s))
                d = d_true + random.gauss(0, SIGMA)
                if random.random() < OUTLIER_P:
                    d += random.choice((-1, 1)) * OUTLIER_M
                d = max(d, 0.0)
                msg = out.named_value_float_encode(
                    int((now - t0) * 1000) & 0xFFFFFFFF, f"d{s}".encode(), d)
                write_serial(msg.pack(out))
                n_uwb += 1
                last[s] = (d_true, d)
                log_row("t", s, d_true)
                log_row("n", s, d)
        time.sleep(0.003)


def stats():
    while True:
        time.sleep(2)
        dist = "  ".join(f"d{s}: vrai={v[0]:.2f} uwb={v[1]:.2f}" for s, v in sorted(last.items()))
        print(f"ESP32->drone: {n_up} o | drone->ESP32: {n_down} o | UWB: {n_uwb} | {dist}")
        with llock:
            log_f.flush()


threading.Thread(target=serial_to_tcp, daemon=True).start()
threading.Thread(target=feeder, daemon=True).start()
threading.Thread(target=stats, daemon=True).start()
print(f"Pont : {COM}@{BAUD} <-> tcp {HOST}:{ESP_PORT} (drone {ME}) + UWB simule")

while True:
    if sock is None:
        connect()
    try:
        data = sock.recv(2048)
        if not data:
            print("Connexion fermee par SITL")
            drop()
            continue
        n_down += len(data)
        write_serial(data)
    except socket.timeout:
        pass
    except OSError:
        drop()