CPP=g++
CFLAGS=-l

all: hello_world


hello_world: hello_world.o
	$(CPP) -o hello hello_world.o

hello_world.o: hello_world.cpp
	$(CPP) -c hello_world.cpp
