import serial

#setting up a serial connection (port and baud rate)
ser = serial.Serial('/dev/cu.usbmodem11101', 9600, timeout=1) 

while True:
    line = ser.readline().decode('utf-8').strip()  # Read and decode serial data
    print(f"received: {line}")