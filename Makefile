CC = g++
override CFLAGS += -Og -g -Wall

.PHONY: clean

usque.o: src/usque/usque.cpp
	$(CC) $(CFLAGS) -c -Ilib/ $^ -o out/usque.o

%.o: src/usque/functions/%.cpp
	$(CC) $(CFLAGS) -c -Ilib/ $^ -o out/$@

clean:
	rm -rf **.o out/** 