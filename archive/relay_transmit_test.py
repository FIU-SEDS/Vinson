import serial as sl

arduino = sl.Serial(port='COM8', baudrate='9600', timeout=0.1)

message_write = input('Write message: ')
arduino.write(bytes(message_write, 'utf-8')) # writes to arduino

message_read = arduino.readline()
message_read = message_read.decode('utf-8')

print(f'Read message: {message_read}\n')