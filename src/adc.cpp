#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>

////////////////////////////////////
// Taken from the LTC186* library

#ifndef _BV
#define _BV(a) (1<<(a))
#endif

#define LTC186X_CONFIG_SINGLE_END 7
#define LTC186X_CONFIG_ODD        6
#define LTC186X_CONFIG_S1         5
#define LTC186X_CONFIG_S0         4
#define LTC186X_CONFIG_COM        3
#define LTC186X_CONFIG_UNI        2
#define LTC186X_CONFIG_SLP        1

#define LTC186X_CHAN_DIFF_0P_1N    (0)
#define LTC186X_CHAN_DIFF_2P_3N    (_BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_DIFF_4P_5N    (_BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_DIFF_6P_7N    (_BV(LTC186X_CONFIG_S1) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_DIFF_1P_0N    (_BV(LTC186X_CONFIG_ODD))
#define LTC186X_CHAN_DIFF_3P_2N    (_BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_DIFF_5P_4N    (_BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_DIFF_7P_6N    (_BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S1) | _BV(LTC186X_CONFIG_S0))

#define LTC186X_CHAN_DIFF_0P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END))
#define LTC186X_CHAN_DIFF_1P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD))
#define LTC186X_CHAN_DIFF_2P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_DIFF_3P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_DIFF_4P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_DIFF_5P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_DIFF_6P_7COM  (_BV(LTC186X_CONFIG_COM) | _BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S1)  | _BV(LTC186X_CONFIG_S0))

#define LTC186X_CHAN_SINGLE_0P     (_BV(LTC186X_CONFIG_SINGLE_END))
#define LTC186X_CHAN_SINGLE_1P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD))
#define LTC186X_CHAN_SINGLE_2P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_SINGLE_3P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_SINGLE_4P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_SINGLE_5P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S1))
#define LTC186X_CHAN_SINGLE_6P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_S1)  | _BV(LTC186X_CONFIG_S0))
#define LTC186X_CHAN_SINGLE_7P     (_BV(LTC186X_CONFIG_SINGLE_END) | _BV(LTC186X_CONFIG_ODD) | _BV(LTC186X_CONFIG_S1) | _BV(LTC186X_CONFIG_S0))

//
////////////////////////////////////

using namespace std;

uint8_t getConfig(uint8_t channel, uint8_t unipolar=1) {
	uint8_t config;
	
	config = unipolar?_BV(LTC186X_CONFIG_UNI):0;
	config |= channel 
		& (_BV(LTC186X_CONFIG_SINGLE_END) 
			| _BV(LTC186X_CONFIG_ODD) 
			| _BV(LTC186X_CONFIG_S1) 
			| _BV(LTC186X_CONFIG_S0)
			| _BV(LTC186X_CONFIG_COM));
	
	return config;
}

void getBits(uint8_t c, char* bits) {
	// convert a byte into a null-terminated char array
	// (bits must be a pointer to an array of length 9)
	for (int i=7; i>=0; --i) {
		bits[i] = (c & (1 << i)) ? '1' : '0';
	}
	bits[8] = '\0';
}

int main() {
	// which bus (0 for "/dev/spidev0.0" or 1 for "/dev/spidev0.1")
	int bus = 0;
	
	// holds result (i.e., error code) from calls to wiringPiSPI
	int res;
	
	// open SPI bus 0 at clock speed 8 MHz - should return a positive integer (a file descriptor)
	res = wiringPiSPISetup(bus, 8000000);
	cout << "SPI setup: " << res << endl;
	
	// sleep for 1 ms (FIXME: unnecessary?)
	usleep(5000);
	
	// command - it is one byte (bits say channel number, properties, etc.)
	uint8_t cmd = getConfig(LTC186X_CHAN_SINGLE_0P);
	
	// print bits in command
	char bits[9];
	getBits(cmd, bits);
	bool sleeping = cmd & _BV(LTC186X_CONFIG_SLP);
	cout << "SPI cmd : "
		 << int(cmd) << " (int) : "
		 << bits << " (bits) : " 
		 << (sleeping ? "sleeping" : "not sleeping")
		 << endl;
	
	// create buffer with bytes [command, "don't use"]
	uint8_t buf[2];
	buf[0] = cmd;
	buf[1] = 0;
	
	// send command (bus, buffer, buffer length) to trigger ADC conversion -
	// this will overwrite the buffer with data from read
	res = wiringPiSPIDataRW(bus, buf, 2);
	cout << "SPI RW : " << res << " : ";
	cout << "[" << int(buf[0]) << ", " << int(buf[1]) << "] (int) : ";
	getBits(buf[0], bits);
	cout << "[" << bits << ", ";
	getBits(buf[1], bits);
	cout << bits << "] (bits)" << endl;
	
	// sleep for 1 ms (FIXME: unnecessary?)
	usleep(5000);
	
	// send command (bus, buffer, buffer length) to get ADC data - again,
	// this will overwrite the buffer with data from read
	res = wiringPiSPIDataRW(bus, buf, 2);
	cout << "SPI RW : " << res << " : ";
	cout << "[" << int(buf[0]) << ", " << int(buf[1]) << "] (int) : ";
	getBits(buf[0], bits);
	cout << "[" << bits << ", ";
	getBits(buf[1], bits);
	cout << bits << "] (bits)" << endl;
	
	// exit with no error
	return 0;
}
