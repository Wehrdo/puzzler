CPP=g++
CFLAGS=-I../opencv/build
LIBS=`pkg-config --libs opencv`

all: hello_world DisplayImage

DisplayImage: DisplayImage.cpp
	$(CPP) $(CFLAGS) $(LIBS) -o $@ $<

hello_world: hello_world.o
	$(CPP) -o hello hello_world.o

hello_world.o: hello_world.cpp
	$(CPP) -c hello_world.cpp

clean: 
	rm hello;
	rm DisplayImage
