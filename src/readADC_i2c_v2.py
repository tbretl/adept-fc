import time
import smbus
channel = 1
bus = smbus.SMBus(1)
address = 0x53

bus.write_byte_data(address,0x07,0X00)
bus.write_byte_data(address,0x07,0X01)
bus.write_byte_data(address,0x07,0X00)
time.sleep(0.5)

while True:
	data = bus.read_i2c_block_data(address,0x00,7)
	print("ADC data : %d" %data)
	time.sleep(0.5)
	
	
