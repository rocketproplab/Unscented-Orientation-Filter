# g++ options: 
#	-c means only run preprocess, compile, and assemble steps (no linking?)
#	-g means generate source-level debug info
#	-fPIC generates position independent code and is necessary for compiling
#		a library
# a previous version of this Makefile used g++-10
JAVA_HOME:=/Library/Java/JavaVirtualMachines/adoptopenjdk-8.jdk/Contents/Home/
.PHONY: all

all: USQUE.h USQUE.class Main.class libUSQUE.dylib native_filter.o params

USQUE.h USQUE.class: USQUE.java
	javac -h . USQUE.java

Main.class: Main.java
	javac Main.java

libUSQUE.dylib: filter_step.o
	g++ -shared -o libUSQUE.dylib filter_step.o

filter_step.o: filter_step.cpp USQUE.h
	g++ -Wall -I"${JAVA_HOME}/include" -I"${JAVA_HOME}/include/darwin" -c -g -fPIC native_filter.cpp

params: write_params.cpp lib/json.hpp
	g++ -Wall -o write_params write_params.cpp
	./write_params

clean:
	rm -f USQUE.h libUSQUE.dylib Main.class USQUE.class native_filter.o write_params

