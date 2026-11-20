from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14552')   # adapte selon le tableau
hb = m.wait_heartbeat(timeout=10)
print("Heartbeat reçu :", hb)   