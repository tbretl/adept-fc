#include <SPI.h>
#include <Wire.h>
#include <Ard1863.h>

#define SerialConsole Serial
#define CSPIN_TOP     3
#define CSPIN_BOT     8
#define UNIPOLAR      1

Ard186x ard186xboard_top;
Ard186x ard186xboard_bot;

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
}

void loop() {

  //sample all the channels, send data over serial 
  SerialConsole.print(micros()); //timestamp
  SerialConsole.print(','); 
  SerialConsole.print(ard186xboard_top.ltc186xReadAndChangeChannel(LTC186X_CHAN_SINGLE_0P, UNIPOLAR));
  SerialConsole.print(','); 
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
  SerialConsole.println();

  delay(1000);

}
