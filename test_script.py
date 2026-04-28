import serial
import struct
import time
import zlib
import csv
import os
from datetime import datetime
import time
#Configuration
SERIAL_PORT = '/dev/controls_esp'  # Change to '/dev/ttyS0' if using GPIO pins
BAUD_RATE = 115200            # Make sure to match
TIMEOUT_THRESHOLD = 2.0  # Seconds of silence before starting a new file
RECONNECT_DELAY = 1.0    # Seconds to wait before trying to reconnect the port

# --- PROTOCOL DEFINITION ---
# Header: Start(1), Start(1), Type(2), Seq(4) = 8 Bytes
HEADER_FMT = '<2BHI' 
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Payload: 50 Floats = 200 Bytes
SENSOR_NAMES = [
    "time_ms", "engine_count", "gear_count", "engine_rpm",
    "secondary_rpm", "filtered_engine_rpm", "filtered_secondary_rpm", "target_rpm",
    "engine_rpm_error", "velocity_command", "ecvt_velocity", "ecvt_pos", "ecvt_iq", "inbound_limit_switch", "outbound_limit_switch", "engage_limit_switch"
]
PAYLOAD_FLOAT_COUNT = len(SENSOR_NAMES)
PAYLOAD_FMT = f'<{PAYLOAD_FLOAT_COUNT}f'
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

# Footer: CRC32 (4 Bytes
FOOTER_FMT = '<I'
FOOTER_SIZE = struct.calcsize(FOOTER_FMT)

# Total Packet Size
PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE + FOOTER_SIZE

SYNC_1 = 0xFA
SYNC_2 = 0xCE


# Names for your CSV columns (Must match C++ enum order!)

# Fill the rest with generic names if you have 50
while len(SENSOR_NAMES) < PAYLOAD_FLOAT_COUNT:
    SENSOR_NAMES.append(f"Sensor_{len(SENSOR_NAMES)}")

def main():
    print(f"--- BAJA TELEMETRY LOGGER ---")
    print(f"Port: {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"Packet Size: {PACKET_SIZE} bytes")
    print("Waiting for sync...")

   
    ser = None        
    packets_received = 0
    last_print_time = time.time()
    crc_errors = 0
    while True: # Main Reconnection Loop
        try:
            if ser is None or not ser.is_open:
                print(f"Connecting to {SERIAL_PORT}...")
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False, rtscts=False)
                ser.dtr = False
                ser.rts = False
                ser.reset_input_buffer()
                print("Connected!")

            # Create a new file for this session
            log_filename = f"baja_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            print(f"\n--- STARTING NEW LOG: {log_filename} ---")
            with open(log_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["PC_Timestamp", "Seq_Num", "Packet_Type"] + SENSOR_NAMES)

                last_packet_time = time.time()
                packets_received = 0
                crc_errors = 0
                last_print_time = time.time()
                while True:
                    if time.time() - last_packet_time > TIMEOUT_THRESHOLD:
                        if packets_received > 0:
                            print(f"\nData timeout (> {TIMEOUT_THRESHOLD}s). Closing file.")
                            break # Break inner loop to start a new file
                    
                    if ser.read(1) != bytes([SYNC_1]):
                        continue 
                    if ser.read(1) != bytes([SYNC_2]):
                        continue
                    last_packet_time = time.time()
                    bytes_needed = PACKET_SIZE - 2
                    raw_data = ser.read(bytes_needed)
                    print(len(raw_data), bytes_needed)
                    if len(raw_data) != bytes_needed:
                        print("Incomplete packet, resetting sync...")
                        continue

                    
                    full_packet_bytes = bytes([SYNC_1, SYNC_2]) + raw_data
                    
                    data_to_check = full_packet_bytes[:-4]
                    
                    received_crc = struct.unpack(FOOTER_FMT, raw_data[-4:])[0]
                    
                    calculated_crc = zlib.crc32(data_to_check) & 0xFFFFFFFF

                    if calculated_crc != received_crc:
                        crc_errors += 1
                        print(f"[CRC FAIL] Calc: {calculated_crc:08X} != Recv: {received_crc:08X}")
                        continue # 

                    header_vals = struct.unpack('<HI', raw_data[0:6])
                    pkt_type = header_vals[0]
                    seq_num = header_vals[1]

                    payload_bytes = raw_data[6 : 6 + PAYLOAD_SIZE]
                    floats = struct.unpack(PAYLOAD_FMT, payload_bytes)

                    writer.writerow([time.time(), seq_num, pkt_type] + list(floats))
                    packets_received += 1

                    
                    if time.time() - last_print_time > 1.0:
                        fps = packets_received / (time.time() - last_print_time + 0.001)
                        print(f"\r[REC] Rate: {fps:.1f} Hz | Seq: {seq_num} | Errors: {crc_errors} | Steer: {floats[1]:.2f}", end='')
                        
                        packets_received = 0
                        last_print_time = time.time()

        except (serial.SerialException, OSError) as e:
            print(f"\nConnection lost or Port Error: {e}")
            if ser:
                ser.close()
            ser = None
            time.sleep(RECONNECT_DELAY)
        
        except KeyboardInterrupt:
            print("\nStopping logger...")
            if ser:
                ser.close()
            break
        
        except Exception as e:
            print(f"\nUnexpected Error: {e}")
            time.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    main()
