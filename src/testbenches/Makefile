CC = g++
override CFLAGS += -Og -g -Wall
ROOT := ../..
LIB  = $(ROOT)/lib
.PHONY: clean

testbench: test/testbench-sim.o
	$(CC) $(CFLAGS) -I $(LIB) -I $(ROOT)/src/usque $^ -o test/testbench

test/testbench-sim.o: testbench-sim.cpp
	$(CC) $(CFLAGS) -c -I $(LIB) -I $(ROOT)/src/usque $^ -o $@

clean:
	rm -rf **.o out/** 