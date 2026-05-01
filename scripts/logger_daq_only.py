import serial
import struct
import time
import zlib
import csv
import os
from datetime import datetime
import threading
import queue
SERIAL_PORT      = '/dev/daq_esp'
BAUD_RATE        = 115200
TIMEOUT_THRESHOLD = 2.0
RECONNECT_DELAY  = 1.0
LORA_PORT = '/dev/lora_esp'
LORA_BAUD = 57600
LORA_STRUCT_FORMAT = '<i i'            


SYNC_1 = 0xFA
SYNC_2 = 0xCE

HEADER_FMT  = '<HI'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
FOOTER_SIZE = 4

DAQ_FIELDS = ["time_ms", "shock_rl_mm", "shock_rr_mm", "shock_rl_raw", "shock_rr_raw", "longitude", "latitude", "mps", "heading_deg", "brake_pressure_front_psi", "brake_pressure_front_raw", "brake_pressure_back_psi", "brake_pressure_back_raw"]
DAQ_FMT    = f'<{len(DAQ_FIELDS)}f'
DAQ_SIZE   = struct.calcsize(DAQ_FMT)


def lora_handler():
    lora_ser = None
    while True:
        if lora_ser is None or not lora_ser.is_open:
            try:
                # We use a short timeout so we don't hang the thread too long
                lora_ser = serial.Serial(LORA_PORT, LORA_BAUD, timeout=0.05)
                lora_ser.dtr = lora_ser.rts = False
            except Exception:
                lora_ser = None
                time.sleep(2) # Don't spam the CPU if the device is missing
                continue

        try:
            lat, lon = lora_queue.get(timeout=1.0)
            
            try:
                packet = struct.pack(LORA_STRUCT_FORMAT, int(lat * 1e7), int(lon * 1e7))
                lora_ser.write(packet)
                lora_ser.flush()
            except Exception:
                lora_ser.close()
                lora_ser = None
        except queue.Empty:
            continue

def read_packet(ser):
    if ser.read(1) != bytes([SYNC_1]):
        return None
    if ser.read(1) != bytes([SYNC_2]):
        return None

    header_raw = ser.read(HEADER_SIZE)
    if len(header_raw) != HEADER_SIZE:
        return None
    pkt_type, seq_num = struct.unpack(HEADER_FMT, header_raw)

    payload_raw = ser.read(DAQ_SIZE)
    if len(payload_raw) != DAQ_SIZE:
        return None

    checksum_raw = ser.read(FOOTER_SIZE)
    if len(checksum_raw) != FOOTER_SIZE:
        return None

    received_crc   = struct.unpack('<I', checksum_raw)[0]
    full_packet    = bytes([SYNC_1, SYNC_2]) + header_raw + payload_raw + checksum_raw
    calculated_crc = zlib.crc32(full_packet[:-4]) & 0xFFFFFFFF

    if calculated_crc != received_crc:
        print(f"[CRC FAIL] calc={calculated_crc:08X} recv={received_crc:08X}")
        return None

    values = struct.unpack(DAQ_FMT, payload_raw)
    return seq_num, values


def main():
    print(f"--- DAQ LOGGER ---")
    print(f"Port: {SERIAL_PORT} @ {BAUD_RATE}")
    ser = None
    lora_ser = None
    if "lat" in DAQ_FIELDS and "lon" in DAQ_FIELDS:
        LORA_LAT_INDEX = DAQ_FIELDS.index("lat")
        LORA_LONG_INDEX = DAQ_FIELDS.index("lon")
    lora_thread = threading.Thread(target=lora_handler, daemon=True)
    lora_thread.start()
    while True:
        try:
            if ser is None or not ser.is_open:
                print(f"Connecting to {SERIAL_PORT}...")
                ser = serial.Serial()
                ser.port     = SERIAL_PORT
                ser.baudrate = BAUD_RATE
                ser.timeout  = 0.1
                ser.dsrdtr   = False
                ser.rtscts   = False
                ser.dtr      = False
                ser.rts      = False
                ser.open()
                time.sleep(0.1)
                ser.reset_input_buffer()
                print("Connected!")

            if lora_ser is None or not lora_ser.is_open:
                # print(f"Connecting to {CONTROLS_PORT}...")
                # ser = serial.Serial(CONTROLS_PORT, CONTROLS_BAUD, timeout=0.1)
                # ser.dtr = False
                # ser.rts = False
                # ser.reset_input_buffer()
                lora_ser = serial.Serial(LORA_PORT, LORA_BAUD, timeout=0.1)
                lora_ser.dtr = False
                lora_ser.rts = False
                lora_ser.reset_input_buffer()

            os.makedirs("logs/daq", exist_ok=True)
            log_filename = f"logs/daq/log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            print(f"Logging to {log_filename}")

            with open(log_filename, 'w', newline='') as f:
                writer      = csv.writer(f)
                writer.writerow(["pc_timestamp", "seq_num"] + DAQ_FIELDS)

                last_packet = time.time()
                packets     = 0
                crc_errors  = 0
                last_print  = time.time()

                while True:
                    if time.time() - last_packet > TIMEOUT_THRESHOLD and packets > 0:
                        print(f"Timeout — closing file.")
                        break

                    result = read_packet(ser)
                    if result is None:
                        continue

                    seq_num, values = result
                    last_packet = time.time()

                    writer.writerow([time.time(), seq_num] + list(values))
                    packets += 1

                    if time.time() - last_print > 1.0:
                        if "lat" in DAQ_FIELDS and "lon" in DAQ_FIELDS:
                            try:
                                if not lora_queue.empty():
                                    lora_queue.get_nowait()
                                
                                lora_queue.put_nowait((values[LORA_LAT_INDEX], values[LORA_LONG_INDEX]))
                            except Exception:
                                pass
                        print(f"{packets} pkts/s | seq={seq_num} | crc_errors={crc_errors}")
                        packets   = 0
                        last_print = time.time()

        except (serial.SerialException, OSError) as e:
            print(f"Connection error: {e}")
            if ser:
                ser.close()
            ser = None
            time.sleep(RECONNECT_DELAY)

        except KeyboardInterrupt:
            print("Stopping.")
            if ser:
                ser.close()
            break

        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(RECONNECT_DELAY)


if __name__ == "__main__":
    main()