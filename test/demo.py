#
# Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
#
# Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
#

import serial
import time

portname = '/dev/serial0'
baudrate = 115200

def test_suspend(port):
	# read 10 times
	for i in range(10):
		print(port.readline())
	# pause async outputs
	print('$VNASY,0*XX\r\n'.encode())
	port.write('$VNASY,0*XX\r\n'.encode())
	# read 5 times (should timeout each time)
	for i in range(5):
		print(port.readline())
	# resume async outputs
	port.write('$VNASY,1*XX\r\n'.encode())
	# read 10 times
	for i in range(10):
		print(port.readline())

def test_timing(port, iters=400):
	lines = []
	tstart = time.time()
	for i in range(iters):
		lines.append(port.readline())
	tstop = time.time()
	tdiff = tstop - tstart

	for l in lines:
		print(l)

	print('\n')
	print(tdiff)
	print(iters / tdiff)
	#print(f`====\n  elapsed time for {iters} iters: {tdiff:.2f} seconds ({iters / tdiff:.2f} Hz)`)

def test_checksum(port):
	line = port.readline()
	print(line)
	print(line[1:-5])
	cksum = 0
	for b in line[1:-5]:
		cksum ^= b
	print(cksum, hex(cksum), int(line[-4:-2], 16))

def main():
	# open serial port
	port = serial.Serial(portname, baudrate, timeout=1, write_timeout=1)

	#test_suspend(port)
	#test_timing(port)
	test_checksum(port)

	# close serial port
	port.close()

if __name__ == "__main__":
	main()
