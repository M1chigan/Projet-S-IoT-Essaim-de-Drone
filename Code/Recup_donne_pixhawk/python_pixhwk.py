from pymavlink import mavutil
import time

# --- CONFIGURATION ---
# Replace with your actual serial port.
# Windows examples: 'COM3', 'COM4'
# Linux/Mac examples: '/dev/ttyUSB0', '/dev/ttyACM0'
SERIAL_PORT = r'\\.\COM11'

# Use 115200 for direct USB connection, or 57600 for TELEM ports via FTDI
BAUD_RATE = 115200 

def main():
    print(f"Connecting to Pixhawk on {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    # Initialize the serial connection
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

# Wait for the first heartbeat to confirm communication is established
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received! Connection successful.")

    print("Requesting specific MAVLink messages...")

    # Dictionary of messages to request and their desired frequency in Hz
    messages_to_request = {
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5, # EKF Position (5 Hz)
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 5,         # Raw GPS (5 Hz)
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU: 10,         # IMU/Acceleration (10 Hz)
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10            # Attitude / Orientation (10 Hz)
    }

    # Send a COMMAND_LONG to request each message specifically
    for msg_id, frequency_hz in messages_to_request.items():
        interval_us = 1e6 / frequency_hz # Convert Hz to microseconds
        
        master.mav.command_long_send(
            master.target_system, 
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
            0,               # Confirmation
            msg_id,          # Param 1: The MAVLink message ID
            interval_us,     # Param 2: Interval in microseconds
            0, 0, 0, 0, 0    # Params 3-7 (unused)
        )

    print("Listening for Data... (Press Ctrl+C to stop)")
    
    try:
        while True:
            # Wait for any of these specific messages
            msg = master.recv_match(
                type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'SCALED_IMU', 'ATTITUDE'], 
                blocking=True
            )
            
            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == 'GLOBAL_POSITION_INT':
                lat = msg.lat / 1e7      
                lon = msg.lon / 1e7      
                alt = msg.alt / 1000.0   
                rel_alt = msg.relative_alt / 1000.0 
                
                print(f"[EKF POS] Lat: {lat}, Lon: {lon}, Alt: {alt}m, Rel Alt: {rel_alt}m")

            elif msg_type == 'GPS_RAW_INT':
                raw_lat = msg.lat / 1e7
                raw_lon = msg.lon / 1e7
                raw_alt = msg.alt / 1000.0
                sats = msg.satellites_visible
            
                print(f"[RAW GPS] Lat: {raw_lat}, Lon: {raw_lon}, Alt: {raw_alt}m, Satellites: {sats}")

            elif msg_type == 'SCALED_IMU':
                acc_x = msg.xacc
                acc_y = msg.yacc
                acc_z = msg.zacc
                
                print(f"[IMU ACC] X: {acc_x}mG, Y: {acc_y}mG, Z: {acc_z}mG")

            elif msg_type == 'ATTITUDE':
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw
                
                # Format to 3 decimal places for readability
                print(f"[ATTITUDE] Roll: {roll:.3f} rad, Pitch: {pitch:.3f} rad, Yaw: {yaw:.3f} rad")
                
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nTest terminated.")
if __name__ == '__main__':
    main()