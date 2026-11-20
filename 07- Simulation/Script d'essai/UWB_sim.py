"""
uwb_sim.py (Ubuntu) : calcule des distances "UWB" entre drones SITL.

Source : SERIAL2 de chaque instance (TCP 5763, 5773, 5783), donc SANS toucher
a SERIAL1 (ESP32) ni a MAVProxy.
Position : SIMSTATE (verite terrain, lat/lon) si disponible, sinon
GLOBAL_POSITION_INT (EKF). L'altitude vient de GLOBAL_POSITION_INT.
"""
import math, random, time
from pymavlink import mavutil

mav = mavutil.mavlink
DRONES = {1: 5763, 2: 5773, 3: 5783}      # SYSID -> port TCP (SERIAL2)
RATE_HZ = 10                              # frequence des messages demandes
SIGMA = 0.10                              # bruit UWB (m) : DW1000 ~ +-10 cm
OUTLIER_P = 0.02                          # proba d'une mesure aberrante (NLOS)
OUTLIER_M = 1.5                           # amplitude de l'aberration (m)
M_PER_DEG = 111319.5

links = {s: mavutil.mavlink_connection(f"tcp:127.0.0.1:{p}",
                                       source_system=250, source_component=190)
         for s, p in DRONES.items()}


def request(s, msgid):
    links[s].mav.command_long_send(s, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   msgid, int(1e6 / RATE_HZ), 0, 0, 0, 0, 0)


for s, l in links.items():
    l.wait_heartbeat(timeout=10)
    request(s, 164)      # SIMSTATE
    request(s, 33)       # GLOBAL_POSITION_INT
print("Liens ouverts :", sorted(links))

latlon = {}              # s -> (lat, lon) en degres
alt = {}                 # s -> altitude AMSL en m
origin = None            # (lat0, lon0) commun a tous les drones
use_sim = {}             # s -> True des qu'un SIMSTATE est recu


def update():
    global origin
    for s, l in links.items():
        while True:
            m = l.recv_match(blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == "SIMSTATE":
                latlon[s] = (m.lat / 1e7, m.lng / 1e7)
                use_sim[s] = True
            elif t == "GLOBAL_POSITION_INT":
                alt[s] = m.alt / 1000.0
                if not use_sim.get(s):
                    latlon[s] = (m.lat / 1e7, m.lon / 1e7)
            if origin is None and 1 in latlon:
                origin = latlon[1]


def xyz(s):
    """Position (nord, est, haut) en metres dans le repere commun."""
    lat, lon = latlon[s]
    n = (lat - origin[0]) * M_PER_DEG
    e = (lon - origin[1]) * M_PER_DEG * math.cos(math.radians(origin[0]))
    return n, e, alt.get(s, 0.0)


def true_distance(a, b):
    return math.dist(xyz(a), xyz(b))


def uwb_distance(a, b):
    """Distance telle que la mesurerait un DW1000 (bruit + aberrations)."""
    d = true_distance(a, b) + random.gauss(0, SIGMA)
    if random.random() < OUTLIER_P:
        d += random.choice((-1, 1)) * OUTLIER_M
    return max(d, 0.0)


if __name__ == "__main__":
    t_print = 0
    while True:
        update()
        if origin and all(s in latlon for s in DRONES) and time.time() - t_print > 0.5:
            t_print = time.time()
            pairs = [(1, 2), (1, 3), (2, 3)]
            print("  ".join(f"d{a}{b}: vrai={true_distance(a, b):6.2f} "
                            f"uwb={uwb_distance(a, b):6.2f}" for a, b in pairs))
        time.sleep(0.005)