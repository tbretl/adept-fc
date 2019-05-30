CXX = g++
CXXFLAGS = -Wall -g
NAVIO = ../Navio2/C++/Navio
INCS = -I./message_types -I$(NAVIO) -L$(NAVIO)  
LIBS = -lzcm -lnavio 
BIN = ./bin
SRC = ./src
TST = ./test

all: adc vnins pwm_out scribe rc_in adept_fc autopilot hitl monitor

demos: subber demo udp_send udp_receive

adc: $(SRC)/adc.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/adc.cpp $(SRC)/rs232.c -o $(BIN)/adc

vnins: $(SRC)/vnins.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/vnins.cpp $(SRC)/rs232.c -o $(BIN)/vnins

subber: $(TST)/subber.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(TST)/subber.cpp -o $(BIN)/subber

demo: $(TST)/demo.cpp
	$(CXX) $(CXXFLAGS) -I./src/ $(TST)/demo.cpp $(SRC)/rs232.c -o $(BIN)/demo

pwm_out: $(SRC)/PWM_out_main.cpp 
	$(CXX) $(CXXFLAGS) $(SRC)/PWM_out_main.cpp $(LIBS) $(INCS) -o $(BIN)/pwm_out

scribe: $(SRC)/scribe_main.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/scribe_main.cpp -o $(BIN)/scribe 

rc_in: $(SRC)/RC_in_main.cpp
	$(CXX) $(CXXFLAGS) $(SRC)/RC_in_main.cpp $(LIBS) $(INCS) -o $(BIN)/rc_in 

adept_fc: $(SRC)/adept_fc_main.cpp 
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/adept_fc_main.cpp -o $(BIN)/adept_fc 

autopilot: $(SRC)/autopilot_main.cpp 
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/autopilot_main.cpp -o $(BIN)/autopilot 

hitl: $(SRC)/interface.cpp 
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) -lboost_system -lpthread $(SRC)/interface.cpp -o $(BIN)/hitl 

monitor: $(SRC)/monitor.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) $(SRC)/monitor.cpp -o $(BIN)/monitor

udp_receive: $(TST)/udp_receive.cpp 
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) -lboost_system -lpthread $(TST)/udp_receive.cpp -o $(BIN)/udp_receive 

udp_send: $(TST)/udp_sender.cpp 
	$(CXX) $(CXXFLAGS) -lboost_system -lpthread $(TST)/udp_sender.cpp -o $(BIN)/udp_send 
	

clean:
	rm -f $(BIN)/adc
	rm -f $(BIN)/vnins
	rm -f $(BIN)/subber
	rm -f $(BIN)/demo
	rm -f $(BIN)/pwm_out
	rm -f $(BIN)/scribe
	rm -f $(BIN)/rc_in
	rm -f $(BIN)/udp_receive
	rm -f $(BIN)/adept_fc 
	rm -f $(BIN)/autopilot
	rm -f $(BIN)/hitl
	rm -f $(BIN)/monitori
	rm -f $(BIN)/udp_send

