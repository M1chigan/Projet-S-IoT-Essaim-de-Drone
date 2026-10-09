#include <Arduino.h>
#include <HardwareSerial.h>
// Include the standard C/C++ MAVLink library
#include <mavlink.h> 

// Initialize HardwareSerial for UART2 (RX=16, TX=17 by default on most ESP32 boards)
HardwareSerial PixhawkSerial(2); 

const int PIXHAWK_BAUDRATE = 57600;
bool streams_requested = false;

// Function to request a specific MAVLink message at a given frequency
void request_mavlink_message(uint16_t message_id, float frequency_hz) {
    uint32_t interval_us = (uint32_t)(1000000.0 / frequency_hz);
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack a COMMAND_LONG message to request the data stream
    mavlink_msg_command_long_pack(
        255, 1,                  // System ID and Component ID of the ESP32
        &msg,
        1, 1,                    // Target System and Component (Pixhawk is usually 1, 1)
        MAV_CMD_SET_MESSAGE_INTERVAL, 
        0,                       // Confirmation
        message_id,              // Param 1: The requested message ID
        interval_us,             // Param 2: The requested interval in microseconds
        0, 0, 0, 0, 0            // Params 3-7 (unused)
    );
    
    // Translate the message to a byte buffer and send it over the serial port
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    PixhawkSerial.write(buf, len);
}

void setup() {
    // Serial for USB debugging
    Serial.begin(115200);
    
    // Serial for Pixhawk communication (TELEMETRY port speed is usually 57600)
    PixhawkSerial.begin(PIXHAWK_BAUDRATE, SERIAL_8N1, 16, 17);
    
    Serial.println("ESP32 Ready. Waiting for Pixhawk Heartbeat...");
}

void loop() {
    mavlink_message_t msg;
    mavlink_status_t status;

    // Read bytes while they are available on the UART
    while (PixhawkSerial.available()) {
        uint8_t incomingByte = PixhawkSerial.read();

        // Feed the byte into the MAVLink parser
        if (mavlink_parse_char(MAVLINK_COMM_0, incomingByte, &msg, &status)) {
            
            // Check the message ID once a valid packet is decoded
            switch (msg.msgid) {
                
                case MAVLINK_MSG_ID_HEARTBEAT:
                    // Only request the streams once after receiving the first heartbeat
                    if (!streams_requested) {
                        Serial.println("Heartbeat received! Requesting data streams...");
                        
                        request_mavlink_message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0); // 5 Hz
                        request_mavlink_message(MAVLINK_MSG_ID_GPS_RAW_INT, 5.0);         // 5 Hz
                        request_mavlink_message(MAVLINK_MSG_ID_SCALED_IMU, 10.0);         // 10 Hz
                        request_mavlink_message(MAVLINK_MSG_ID_ATTITUDE, 10.0);           // 10 Hz
                        
                        streams_requested = true;
                        Serial.println("Listening for Data...");
                    }
                    break;

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t gpi;
                    mavlink_msg_global_position_int_decode(&msg, &gpi);
                    
                    Serial.print("[EKF POS] Lat: "); 
                    Serial.print(gpi.lat / 1e7, 7);
                    Serial.print(", Lon: "); 
                    Serial.print(gpi.lon / 1e7, 7);
                    Serial.print(", Alt: "); 
                    Serial.print(gpi.alt / 1000.0);
                    Serial.print("m, Rel Alt: "); 
                    Serial.print(gpi.relative_alt / 1000.0);
                    Serial.println("m");
                    break;
                }
                
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    
                    Serial.print("[RAW GPS] Lat: "); 
                    Serial.print(gps.lat / 1e7, 7);
                    Serial.print(", Lon: "); 
                    Serial.print(gps.lon / 1e7, 7);
                    Serial.print(", Alt: "); 
                    Serial.print(gps.alt / 1000.0);
                    Serial.print("m, Satellites: "); 
                    Serial.println(gps.satellites_visible);
                    break;
                }
                
                case MAVLINK_MSG_ID_SCALED_IMU: {
                    mavlink_scaled_imu_t imu;
                    mavlink_msg_scaled_imu_decode(&msg, &imu);
                    
                    Serial.print("[IMU ACC] X: "); 
                    Serial.print(imu.xacc);
                    Serial.print("mG, Y: "); 
                    Serial.print(imu.yacc);
                    Serial.print("mG, Z: "); 
                    Serial.print(imu.zacc);
                    Serial.println("mG");
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    
                    Serial.print("[ATTITUDE] Roll: "); 
                    Serial.print(att.roll, 3);
                    Serial.print(" rad, Pitch: "); 
                    Serial.print(att.pitch, 3);
                    Serial.print(" rad, Yaw: "); 
                    Serial.print(att.yaw, 3);
                    Serial.println(" rad");
                    break;
                }
            }
        }
    }
}