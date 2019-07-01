//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <SPI.h>
#include <Wire.h>
#include <Ard1863.h>

#define SerialConsole Serial
#define CSPIN_TOP     3
#define CSPIN_BOT     8
#define UNIPOLAR      1

Ard186x ard186xboard_top;
Ard186x ard186xboard_bot;

// On the Arduino Uno, only pins 2 and 3 are usable for interrupts. We
// are already using pin 3 as a chip select for the top ARD186x board,
// so we are forced to use pin 2 as the interrupt.
const byte interruptPin = 2;

// local time in microseconds at which the most recent GPS PPS signal was received
volatile unsigned long lastgpsppstime;

// number of samples since most recent GPS PPS signal
volatile int numsamples;

// sample rate in Hz
const int samplerate = 40;

// sample period in microseconds
const unsigned long sampleperiod = 1000000 / samplerate;

// time of last sample in microseconds
unsigned long sampletime = 0;

// sampled data
unsigned int data[16];

void setup() {

  //Set up serial at XXX bps
  SerialConsole.begin(230400);
  while(!SerialConsole);

  // top board (disabling EEPROM)
  ard186xboard_top.begin(DEVICE_LTC1867, 0x00, CSPIN_TOP);
  ard186xboard_top.setFastSPI(1);                           // FIXME: confirm this does not cause trouble
  ard186xboard_top.ltc186xChangeChannel(LTC186X_CHAN_SINGLE_0P, UNIPOLAR);

  // bottom board (disabling EEPROM)
  ard186xboard_bot.begin(DEVICE_LTC1867, 0x00, CSPIN_BOT);
  ard186xboard_bot.setFastSPI(1);                           // FIXME: confirm this does not cause trouble
  ard186xboard_bot.ltc186xChangeChannel(LTC186X_CHAN_SINGLE_0P, UNIPOLAR);

  lastgpsppstime = micros();
  numsamples = 0;

  // The GPS_PPS signal is low when off, so we use INPUT and not INPUT_PULLUP as the mode.
  pinMode(interruptPin, INPUT);

  // The GPS_PPS signal is synchronized on the rising edge (this is important to get
  // right, because otherwise we will be off by 100 ms, the pulse-width).
  attachInterrupt(digitalPinToInterrupt(interruptPin), setlastppstime, RISING);
}

void setlastppstime() {
  lastgpsppstime = micros();
  numsamples = 0;
}

void loop() {
  unsigned long time_gpspps = micros() - lastgpsppstime;
  if (time_gpspps >= (numsamples * sampleperiod)) {
    // sample all the channels and send data over serial
    SerialConsole.print(time_gpspps);
    SerialConsole.print(",");
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_0P, UNIPOLAR));
    SerialConsole.print(",");
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_1P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_2P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_3P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_4P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_5P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_6P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_7P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_0P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_1P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_2P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_3P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_4P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_5P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_6P, UNIPOLAR));
    SerialConsole.print(',');
    SerialConsole.print(ard186xboard_bot.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_7P, UNIPOLAR));
    SerialConsole.print("\n");

    numsamples += 1;
  }
}
