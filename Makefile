CC = g++
override CFLAGS += -Og -g -Wall

.PHONY: clean

usque.o: src/usque/usque.cpp
	$(CC) $(CFLAGS) -c -Ilib/ $^ -o out/usque.o

%.o: %.cpp
	$(CC) $(CFLAGS) -c -Ilib/ $^ -o $@

clean:
	rm -rf **.o out/** 