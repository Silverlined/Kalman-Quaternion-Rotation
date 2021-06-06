#!/usr/bin/env python3

import serial
import time
import csv
from collections import deque

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# How many sensor samples we want to store
SAMPLE_SIZE = 1000

arduino_serial = None


def get_data():
    global arduino_serial
    if arduino_serial == None:
        arduino_serial = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print("Opened ", arduino_serial.name)
        time.sleep(3)
        arduino_serial.flush()

    # Poll the serial port
    ser_bytes = arduino_serial.readline()

    decoded_bytes = ser_bytes[0 : len(ser_bytes) - 2].decode("utf-8", errors="ignore")
    # print(decoded_bytes)
    if not "Data:" in decoded_bytes:
        return None
    values = decoded_bytes.replace("Data:", "").strip().split(",")
    print(values)
    return values


def main():
    global arduino_serial
    with open("filtered_still.csv", "w", newline="") as csv_file:
        # writer = csv.writer(csv_file)
        headers = ['Acc_X', 'Acc_Y', 'Acc_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z']
        writer = csv.DictWriter(csv_file, delimiter=',', lineterminator='\n',fieldnames=headers)
        writer.writeheader()

        i = 0
        while i < SAMPLE_SIZE:
            values = get_data()
            if values is not None and len(values) == 6:
                writer.writerow({headers[0]: values[0],
                                headers[1]: values[1],
                                headers[2]: values[2],
                                headers[3]: values[3],
                                headers[4]: values[4],
                                headers[5]: values[5],})
                i += 1
        print("Data Collection Finished")

    if csv_file:
        csv_file.close()
    if arduino_serial:
        arduino_serial.close()


if __name__ == "__main__":
    main()
