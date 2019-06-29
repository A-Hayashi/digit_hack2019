
import serial
import time

ser = serial.Serial('/dev/ttyUSB0',115200,timeout=None)

while(1):
	ser.write('m0:50\n'.encode())
	time.sleep(3)
	ser.write('m0:-50\n'.encode())
	time.sleep(3)
	ser.write('m0:0\n'.encode())
	time.sleep(3)
	ser.write('m1:-100\n'.encode())
	time.sleep(3)
	ser.write('m1:0\n'.encode())
	time.sleep(1)
	ser.write('m1:100\n'.encode())
	time.sleep(3)
	ser.write('m1:0\n'.encode())

while(1):
	line = ser.readline()
	print(line) 

ser.close() 

