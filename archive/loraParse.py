import csv

#setting up a serial connection (port and baud rate)
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) use this when recieving live data

input_file = "logs/altitude_data.log"
output_csv = "parsed_data.csv"

with open(output_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Altitude", "RSSI", "Signal"])

#reads the data file and processes each line 
with open(input_file, 'r') as file:
    for line in file: 
        data = line.strip()

        if "Received: +RCV=" in data: 
            clean_data = data.replace("Received: +RCV=", "")
            data_values = clean_data.split(',')
            last_three = data_values[-3:]

            with open(output_csv, mode = 'a', newline = '') as csvfile: 
                writer = csv.writer(csvfile)
                writer.writerow(last_three)

print(f"Data has been written to {output_csv}")