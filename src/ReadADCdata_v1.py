#!/usr/bin/python
import time
import sys
import spidev
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
spi=spidev.SpiDev()
spi.open(0,1)
spi.max_speed_hz=4000000   #16bits=1data, 1.5MSPS needed

SCLK=23
CS=26
MISO=21
GPIO.setup(SCLK,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(MISO,GPIO.IN)
GPIO.output(CS,GPIO.LOW)   #initialize CS

def readADC(clkPin,csPin,misoPin):
   buff=spi.readbytes(4)
   data=0
   for bit in buff:
         data=(data<<8)|int(bit)
         data |= 0xFF
   return data
if __name__=='__main__':
   try:
         while True:
                GPIO.output(CS,GPIO.LOW)  #CS active
                val=readADC(SCLK,CS,MISO)
                volt=val*3.3/(2**14)
                print('ADC value', str(volt))
                GPIO.output(CS,GPIO.HIGH)   #CS inactive
   except KeyboardInterrupt:
         GPIO.cleanup()
         spi.close()
         sys.exit(0)
#########################
