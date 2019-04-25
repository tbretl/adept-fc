import time
import smbus
channel = 1
bus = smbus.SMBus(1)
address = 0x53
reg_write_dac = 0x40
reg_read_dac = 0x00
def bearing255():
	bus.write_byte_data(address,0x07,0x00)
	bear = bus.read_byte_data(address,1)
	return bearing255
print bearing255
def bearing3599():
	data0 = bus.read_byte_data(address,2)
	data1 = bus.read_byte_data(address,3)
	bear = (data0<<8)+data1
	bear = bear/10.0
	return bear
count = 0
while (count <10):
	bearing = bearing3599()
	bear255 = bearing255()
	count = count + 1
print bearing
print bear255
time.sleep(1)

