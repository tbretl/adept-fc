CXX ?= g++
NAVIO = ../../Navio2/C++/Navio 
INCLUDES = -I../../Navio2/C++/Navio -I../message_types/ 

all:
	$(MAKE) -C ../../Navio2/C++/Navio  all
	$(CXX) -std=gnu++11 $(INCLUDES) PWM_out_main.cpp -L$(NAVIO) -lnavio -lzcm -o PWM_Out
