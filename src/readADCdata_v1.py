#!/usr/bin/python
import time
import sys
import spidev
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
spi=spidev.SpiDev()

SCLK=23
CS=26
MISO=21

spi.open(0,1)
spi.max_speed_hz = 4000000
spi.mode = 0
buff = 0
LTC_CONFIG_SLEEP = 1
LTC_CONFIG_UNI = 2
LTC_CONFIG_COM = 3
LTC_CONFIG_S0 = 4
LTC_CONFIG_S1 = 5
LTC_CONFIG_ODD = 6
LTC_CONFIG_SINGLE_END = 7

#LTC_CHAN_SINGLE_0P = bin(LTC_CONFIG_SINGLE_END)
#LTC_CHAN_SINGLE_1P = int(bin(LTC_CONFIG_SINGLE_END)) | int(bin(LTC_CONFIG_ODD))
#LTC_CHAN_SINGLE_2P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S0)
#LTC_CHAN_SINGLE_3P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S0) | bin(LTC_CONFIG_ODD)
#LTC_CHAN_SINGLE_4P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S1)
#LTC_CHAN_SINGLE_5P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S1) | bin(LTC_CONFIG_ODD)
#LTC_CHAN_SINGLE_6P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S1) | bin(LTC_CONFIG_S0) 
#LTC_CHAN_SINGLE_7P = bin(LTC_CONFIG_SINGLE_END) | bin(LTC_CONFIG_S1) | bin(LTC_CONFIG_S0) | bin(LTC_CONFIG_ODD)
currentLTCconfig = None


def Sleep():
	currentLTCconfig |= 0x01
	spi.xfer(currentLTCconfig)
	spi.xfer(0)

	
def b(val):
	return (1<<val)


def wake():
	currentLTCconfig = 0x01
	wasSleep = currentLTCconfig & b(LTC_CONFIG_SLEEP)
	currentLTCconfig &= ~b(LTC_CONFIG_SLEEP)
	spi.xfer([currentLTCconfig])
	spi.xfer([0])	
	if(wasSleep):
		delay(70)
	
	
	

def readADC(clkPin,csPin,misoPin,currentLTCconfig):

	buff=spi.xfer2(currentLTCconfig)
	
	#newbuff = spi.xfer([0])
	#for y in buff:
	#	buff[y]<<8
	#	buff[y] |= 0xFF #& (newbuff[y]>> 4) 
	return buff



	
if __name__=='__main__':
	try:
		while True:
			wake()
			currentLTCconfig = [0x80]
			val=readADC(SCLK,CS,MISO,currentLTCconfig)
			volt=val
			print('ADC value', str(volt))
	except KeyboardInterrupt:
			GPIO.cleanup()
			spi.close()
			sys.exit(0)
