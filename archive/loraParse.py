import csv
import serial
import time

#setting up a serial connection (port and baud rate)
ser = serial.Serial('/dev/cu.usbmodem11101', 9600, timeout=1) 
time.sleep(2)


output_csv = "parsed_data.csv"

with open(output_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Barometer", "Accelerator", "RSSI", "Signal"])

print("Waiting for LoRa data...")
#reads the data file and processes each line 
try:
    while True:
        line = ser.readline().decode('utf-8').strip()  # Read and decode serial data
        if "Received: +RCV=" in line:
            clean_data = line.replace("Received: +RCV=", "")
            data_values = clean_data.split(',')

            if len(data_values) >= 4:  # Ensuring there's enough data
                altitude = data_values[0]  # Altitude 
                rssi = data_values[-2]  # RSSI 
                signal = data_values[-1]  # Signal 

                barometer_accelerometer = data_values[1].split('/')  # Split by '/'
                
                if len(barometer_accelerometer) == 2:
                    barometer, accelerometer = barometer_accelerometer
                else:
                    barometer, accelerometer = "N/A", "N/A"  # handling format errors

                
                with open(output_csv, mode='a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([altitude, rssi, signal, barometer, accelerometer])

                print(f"Logged data: Altitude={altitude}, RSSI={rssi}, Signal={signal}, Barometer={barometer}, Accelerometer={accelerometer}")

except KeyboardInterrupt:
    print("\nStopped receiving data.")
    ser.close()