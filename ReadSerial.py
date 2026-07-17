import serial
import time

# Limit the amount of time this function can run
start = time.time()
current = time.time()
elapsed_time = 0

pico = serial.Serial(port='COM6',baudrate=115200, timeout=.1)

with open( 'data1.txt','w') as f:
	while elapsed_time < 50:
		current = time.time()
		elapsed_time = current - start
		dat = pico.readline()
		dat = dat.decode('utf-8').strip()
		f.write(dat + '\n')
		if len(dat) > 1:
			print(dat)

# pico.write(b'a')

print("While Loop Ended")

import serial

ser = serial.Serial('COMX', 115200)  # Replace COMX with the actual port
with open('data.txt', 'w') as f:
    while True:
        line = ser.readline().decode().strip()
        f.write(line + '\n')
