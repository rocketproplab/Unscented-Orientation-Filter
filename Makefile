CC = gcc
override CFLAGS += -Og -g -Wall

TARGETS = runFilter.o testbench-sim.o filter_step.o

.PHONY: clean

all: $(TARGETS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c -Ilib/ $^ -o $@

clean:
	rm -f **.o  