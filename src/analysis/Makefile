CPP=g++ -std=c++11
CFLAGS=-I../opencv/build -g -ggdb -O0
LIBS=`pkg-config --libs opencv`

all: test

test: test.cpp *.cpp *.hpp
	$(CPP) $(CFLAGS) $(LIBS) -o $@ $<

clean: 
	rm test;
