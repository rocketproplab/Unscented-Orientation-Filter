CC = g++
override CFLAGS += -Og -g -Wall

FUNC_DIR := src/usque/functions
FUNC_NAMES := ${sort ${wildcard ${FUNC_DIR}/*.cpp}}
FUNC_ALL = $(patsubst $(FUNC_DIR)/%.cpp, out/%.o, $(FUNC_NAMES))


.PHONY: clean

names: $(FUNC_NAMES)
	echo $(FUNC_ALL)

all: $(FUNC_ALL)

testbench:
	$(MAKE) -C src/testbenches testbench

usque.o: src/usque/usque.cpp
	$(CC) $(CFLAGS) -c -I lib -I out $^ -o out/usque.o

out/%.o: src/usque/functions/%.cpp
	$(CC) $(CFLAGS) -c -I lib/ -I src/usque $^ -o $@

clean:
	rm -rf **.o out/** 