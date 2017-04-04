CPP=g++ -std=c++11
CPP_FILES := $(wildcard *.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LIBS=`pkg-config --libs opencv`
CFLAGS=-I../opencv/build -g -ggdb -O0

all: dirs test

test: $(OBJ_FILES)
	$(CPP) -o $@ $^ $(LIBS)

obj/%.o: ./%.cpp
	$(CPP) $(CFLAGS) -c -o $@ $<

dirs:
	mkdir -p obj

clean: 
	rm -rf test;
	rm -rf obj;
