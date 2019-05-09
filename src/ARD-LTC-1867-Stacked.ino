/*************************************************************************
Title:    ARD-LTC186X Library Example Arduino Sketch
Authors:  Nathan D. Holmes <maverick@drgw.net>, Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2018 Nathan D. Holmes & Michael D. Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Ard1863.h>


#define SerialConsole Serial

Ard186x ard186xboard1;

void setup() {

  int chipSelectPin = 8; // 3 is good for most things, but Leonardos share SCL with D3.  8 is the other option

  //Set up serial at XXX bps 
  SerialConsole.begin(230400);
  while(!SerialConsole);
  
  // This is the default initialization command MODIFIED TO DISABLE EEPROM with the 0x00 byte
  ard186xboard1.begin(DEVICE_LTC1867, 0x00, chipSelectPin);
  ard186xboard1.setFastSPI(1);
  ard186xboard1.ltc186xChangeChannel(LTC186X_CHAN_SINGLE_0P, 1);
}

void loop() {

  //sample all the channels, send data over serial 
  SerialConsole.print(micros()); //timestamp
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_0P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_1P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_2P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_3P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_4P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_5P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_6P,1));
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard1.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_7P,1));
  SerialConsole.println();

}
