Makefile:
# FIXME - use generic template, add target "clean", etc.

CXX = g++
CXXFLAGS = -Wall -g
INCS = -I../message_types
LIBS = -lzcm
BIN = ../bin

all: vnins subber demo

vnins: vnins.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) vnins.cpp rs232.c -o $(BIN)/vnins

subber: subber.cpp
	$(CXX) $(CXXFLAGS) $(INCS) $(LIBS) subber.cpp -o $(BIN)/subber

demo: demo.cpp
	$(CXX) $(CXXFLAGS) demo.cpp rs232.c -o $(BIN)/demo
