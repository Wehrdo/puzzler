CPP=g++
CFLAGS=-I../opencv/build --std=c++11
LIBS=`pkg-config --libs opencv`

all: hello_world display_image points_test get_points

get_points: get_points.cpp
	$(CPP) $(CFLAGS) $(LIBS) -o $@ get_points.cpp

points_test: points_test.cpp
	$(CPP) $(CFLAGS) $(LIBS) -o $@ points_test.cpp

display_image: display_image.cpp
	$(CPP) $(CFLAGS) display_image.cpp $(LIBS) -o $@

hello_world: hello_world.o
	$(CPP) -o hello hello_world.o

hello_world.o: hello_world.cpp
	$(CPP) -c hello_world.cpp

clean:
	rm hello;
	rm display_image
