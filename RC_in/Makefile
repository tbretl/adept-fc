CXX ?= g++
NAVIO = /home/pi/Navio2/C++/Navio 

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)

INCLUDES = -I /home/pi/Navio2/C++/Navio -I../message_types/ -I$(PIGPIO_PATH)

all:
	$(MAKE) -C /home/pi/Navio2/C++/Navio all
	$(CXX) -std=gnu++11 $(INCLUDES) $(LIB) RC_in_main.cpp -L$(NAVIO) -lzcm -lnavio -o RCInput -lrt -lpthread -lpigpio || $(MAKE) pigpio

