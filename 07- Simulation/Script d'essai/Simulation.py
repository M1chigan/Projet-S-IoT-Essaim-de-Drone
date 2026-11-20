from pymavlink import mavutil
import threading, time, math

mav = mavutil.mavlink
DRONES = [2, 3]
REF = 1                                   # drone 1 (ESP32) : écouté, jamais commandé
CONN = 'udpin:127.0.0.1:14553'            # MAVProxy : output add 127.0.0.1:14555

m = mavutil.mavlink_connection(CONN)
send_lock = threading.Lock()
armed, mode, pos = {}, {}, {}
anchor = None

# ---------- Threads de fond : heartbeat + lecture ----------
def hb():
    while True:
        with send_lock:
            try: m.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception: pass
        time.sleep(1)

def reader():
    while True:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        s, t = msg.get_srcSystem(), msg.get_type()
        if s not in DRONES + [REF]:
            continue
        if t == 'HEARTBEAT':
            armed[s] = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            mode[s] = msg.custom_mode
        elif t == 'GLOBAL_POSITION_INT':
            pos[s] = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000)

threading.Thread(target=hb, daemon=True).start()
threading.Thread(target=reader, daemon=True).start()

# ---------- Envoi d'ordres ----------
def cmd(s, c, *p):
    p = list(p) + [0] * (7 - len(p))
    with send_lock:
        m.mav.command_long_send(s, 1, c, 0, *p)

def set_mode(s, md):
    with send_lock:
        m.mav.set_mode_send(s, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, md)

def goto(s, lat, lon, alt):
    with send_lock:
        m.mav.set_position_target_global_int_send(
            0, s, 1, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000, int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)

def offset_to_latlon(lat0, lon0, dn, de):
    return (lat0 + dn / 111320,
            lon0 + de / (111320 * math.cos(math.radians(lat0))))

# ---------- Formations : décalages (nord, est) en mètres autour de l'ancre ----------
def formation(name, d):
    forms = {
        'line':   {2: (0, -d / 2), 3: (0,  d / 2)},     # alignés est-ouest
        'column': {2: (-d / 2, 0), 3: (d / 2, 0)},      # alignés nord-sud
        'v':      {2: (-d, -d),    3: (-d,  d)},        # en V derrière l'ancre
        'diag':   {2: (0, 0),      3: (d, d)},          # en diagonale
    }
    return forms.get(name)

def get_anchor():
    if REF in pos:
        return pos[REF][:2]
    if 2 in pos:
        return pos[2][:2]
    return None

# ---------- Actions de la console ----------
def do_takeoff(alt):
    for s in DRONES:
        cmd(s, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 250000)   # position à 4 Hz
        set_mode(s, 4)                                          # GUIDED
    time.sleep(1)
    t0 = time.time()
    while time.time() - t0 < 40 and not all(armed.get(s) for s in DRONES):
        for s in DRONES:
            if not armed.get(s):
                cmd(s, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        time.sleep(2)
    if not all(armed.get(s) for s in DRONES):
        print("Armement échoué :", armed, "(GPS prêt ? regarde les PreArm dans MAVProxy)")
        return
    for s in DRONES:
        cmd(s, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    print(f"Décollage à {alt} m envoyé.")

def do_form(name, d, alt):
    offs = formation(name, d)
    if offs is None:
        print("Formations :", ", ".join(['line', 'column', 'v', 'diag']))
        return
    a = get_anchor()
    if a is None:
        print("Aucune position reçue, pas d'ancre.")
        return
    for s in DRONES:
        lat, lon = offset_to_latlon(a[0], a[1], *offs[s])
        goto(s, lat, lon, alt)
    print(f"Formation '{name}' (espacement {d} m, alt {alt} m) envoyée.")

def do_place(s, dn, de, alt):
    a = get_anchor()
    if a is None or s not in DRONES:
        print("Ancre absente ou drone non commandable.")
        return
    lat, lon = offset_to_latlon(a[0], a[1], dn, de)
    goto(s, lat, lon, alt)
    print(f"Drone {s} -> N{dn:+} m, E{de:+} m, alt {alt} m")

def do_status():
    for s in [REF] + DRONES:
        p = pos.get(s)
        txt = f"lat={p[0]:.6f} lon={p[1]:.6f} alt={p[2]:.1f} m" if p else "pas de position"
        print(f"  drone {s}: armé={armed.get(s)} mode={mode.get(s)} {txt}")
    if 2 in pos and 3 in pos:
        dn = (pos[2][0] - pos[3][0]) * 111320
        de = (pos[2][1] - pos[3][1]) * 111320 * math.cos(math.radians(pos[2][0]))
        print(f"  distance 2-3 : {math.hypot(dn, de):.1f} m")

HELP = """Commandes :
  status                         état des drones et distance 2-3
  takeoff [alt]                  GUIDED + armement + décollage (défaut 10 m)
  form <line|column|v|diag> [espacement=10] [alt=10]
  place <2|3> <nord> <est> [alt] position relative à l'ancre (m)
  rtl | land                     retour maison / atterrissage
  help | quit"""

print("Attente des drones...")
t0 = time.time()
while time.time() - t0 < 15 and not all(s in armed for s in DRONES):
    time.sleep(0.5)
print("Drones vus :", sorted(armed))
print(HELP)

while True:
    try:
        parts = input("swarm> ").split()
    except (EOFError, KeyboardInterrupt):
        break
    if not parts:
        continue
    c, a = parts[0], parts[1:]
    try:
        if c == 'status':   do_status()
        elif c == 'takeoff': do_takeoff(float(a[0]) if a else 10)
        elif c == 'form':
            do_form(a[0] if a else '', float(a[1]) if len(a) > 1 else 10,
                    float(a[2]) if len(a) > 2 else 10)
        elif c == 'place':  do_place(int(a[0]), float(a[1]), float(a[2]),
                                     float(a[3]) if len(a) > 3 else 10)
        elif c == 'rtl':
            for s in DRONES: set_mode(s, 6)
        elif c == 'land':
            for s in DRONES: set_mode(s, 9)
        elif c in ('quit', 'exit'): break
        else: print(HELP)
    except (IndexError, ValueError):
        print("Arguments invalides.\n" + HELP)