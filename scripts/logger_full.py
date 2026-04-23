import serial
import struct
import time
import zlib
import csv
import os
from datetime import datetime
import threading

SERIAL_PORTS = {
    'controls': '/dev/controls_esp',
    'daq':      '/dev/daq_esp',
}

BAUD_RATE        = 115200
TIMEOUT_THRESHOLD = 2.0
RECONNECT_DELAY  = 1.0

SYNC_1 = 0xFA
SYNC_2 = 0xCE

HEADER_FMT  = '<HI'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
FOOTER_SIZE = 4

PACKET_TYPE_CONTROLS = 0x01
PACKET_TYPE_DAQ      = 0x02

CONTROLS_FIELDS = [
    "time_ms", "engine_count", "gear_count", "engine_rpm",
    "secondary_rpm", "filtered_engine_rpm", "filtered_secondary_rpm",
    "target_rpm", "engine_rpm_error", "velocity_command",
    "ecvt_velocity", "ecvt_pos", "ecvt_iq",
    "inbound_limit_switch", "outbound_limit_switch", "engage_limit_switch",
]
CONTROLS_FMT  = f'<{len(CONTROLS_FIELDS)}f'
CONTROLS_SIZE = struct.calcsize(CONTROLS_FMT)

DAQ_FIELDS = ["time_ms", "shock_rl_mm", "shock_rr_mm", "shock_rl_raw", "shock_rr_raw"]
DAQ_FMT    = f'<{len(DAQ_FIELDS)}f'
DAQ_SIZE   = struct.calcsize(DAQ_FMT)

PACKET_CONFIGS = {
    PACKET_TYPE_CONTROLS: (CONTROLS_FMT, CONTROLS_SIZE, CONTROLS_FIELDS, "controls"),
    PACKET_TYPE_DAQ:      (DAQ_FMT,      DAQ_SIZE,      DAQ_FIELDS,      "daq"),
}


def read_packet(ser):
    if ser.read(1) != bytes([SYNC_1]):
        return None
    if ser.read(1) != bytes([SYNC_2]):
        return None

    header_raw = ser.read(HEADER_SIZE)
    if len(header_raw) != HEADER_SIZE:
        return None
    pkt_type, seq_num = struct.unpack(HEADER_FMT, header_raw)

    if pkt_type not in PACKET_CONFIGS:
        print(f"Unknown packet type: {pkt_type:#04x}")
        return None

    payload_fmt, payload_size, fields, _ = PACKET_CONFIGS[pkt_type]

    payload_raw = ser.read(payload_size)
    if len(payload_raw) != payload_size:
        return None

    checksum_raw = ser.read(FOOTER_SIZE)
    if len(checksum_raw) != FOOTER_SIZE:
        return None

    received_crc   = struct.unpack('<I', checksum_raw)[0]
    full_packet    = bytes([SYNC_1, SYNC_2]) + header_raw + payload_raw + checksum_raw
    calculated_crc = zlib.crc32(full_packet[:-4]) & 0xFFFFFFFF

    if calculated_crc != received_crc:
        print(f"[CRC FAIL] type={pkt_type:#04x} calc={calculated_crc:08X} recv={received_crc:08X}")
        return None

    values = struct.unpack(payload_fmt, payload_raw)
    return pkt_type, seq_num, fields, values


def run_logger(port_key, serial_port):
    print(f"[{port_key}] Connecting to {serial_port}...")
    ser = None

    while True:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial()
                ser.port     = serial_port
                ser.baudrate = BAUD_RATE
                ser.timeout  = 0.1
                ser.dsrdtr   = False
                ser.rtscts   = False
                ser.dtr      = False
                ser.rts      = False
                ser.open()
                time.sleep(0.1)
                ser.reset_input_buffer()
                print(f"[{port_key}] Connected!")

            os.makedirs(f"logs/{port_key}", exist_ok=True)
            log_filename = f"logs/{port_key}/log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            print(f"[{port_key}] Logging to {log_filename}")

            with open(log_filename, 'w', newline='') as f:
                writer      = None
                last_packet = time.time()
                packets     = 0
                crc_errors  = 0
                last_print  = time.time()

                while True:
                    if time.time() - last_packet > TIMEOUT_THRESHOLD and packets > 0:
                        print(f"[{port_key}] Timeout — closing file.")
                        break

                    result = read_packet(ser)
                    if result is None:
                        continue

                    pkt_type, seq_num, fields, values = result
                    last_packet = time.time()

                    if writer is None:
                        writer = csv.writer(f)
                        writer.writerow(["pc_timestamp", "seq_num", "packet_type"] + fields)

                    writer.writerow([time.time(), seq_num, pkt_type] + list(values))
                    packets += 1

                    if time.time() - last_print > 1.0:
                        print(f"[{port_key}] {packets} pkts/s | seq={seq_num} | crc_errors={crc_errors}")
                        packets   = 0
                        last_print = time.time()

        except (serial.SerialException, OSError) as e:
            print(f"[{port_key}] Connection error: {e}")
            if ser:
                ser.close()
            ser = None
            time.sleep(RECONNECT_DELAY)

        except KeyboardInterrupt:
            print(f"[{port_key}] Stopping.")
            if ser:
                ser.close()
            break

        except Exception as e:
            print(f"[{port_key}] Unexpected error: {e}")
            time.sleep(RECONNECT_DELAY)


if __name__ == "__main__":
    threads = []
    for key, port in SERIAL_PORTS.items():
        t = threading.Thread(target=run_logger, args=(key, port), daemon=True)
        t.start()
        threads.append(t)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down.")