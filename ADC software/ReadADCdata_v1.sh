#!/usr/bin/python
import time
import sys
import spidev
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
spi=spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=24000000   #16bits=1data, 1.5MSPS needed

SCLK=23
CS=24
MISO=21
GPIO.setup(SCLK,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(MISO,GPIO.IN)
GPIO.output(CS,GPIO.HIGH)   #initialize CS

def readADC(clkPin,csPin,misoPin):
   buff=spi.readbytes(2)
   bData=bin(buff[0])[2:].zfill(8)[2:]+bin(buff[1])[2:].zfill(8)
   bData_rev=bData[::-1]
   data=0
   for bit in bData_rev:
         data=(data<<1)|int(bit)
return data
if __name__=='__main__':
   try:
         while True:
                GPIO.output(CS,GPIO.HIGH)  #CS active
                val=readADC(SCLK,CS,MISO)
                volt=val*3.3/(2**14)
                print 'ADC value', str(volt)
                GPIO.output(CS,GPIO.LOW)   #CS inactive
   except KeyboardInterrupt:
         GPIO.cleanup()
         spi.close()
         sys.exit(0)
#########################
