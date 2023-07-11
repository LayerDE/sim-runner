CXX = g++ -Wall -pedantic -std=c++11
CC = gcc -Wall -pedantic -std=c11
MAIN_BINARIES = $(basename $(wildcard *Main.cpp))
CMAIN_BINARIES = $(basename $(wildcard *Main.c))
HEADERS = $(wildcard *.hpp)
CHEADERS = $(wildcard *.h)
SIM_OBJECTS = $(addsuffix .cpp.o, $(basename $(filter-out %Main.cpp, $(wildcard simulator/*.cpp))))
SIM_COBJECTS = $(addsuffix .c.o, $(basename $(filter-out %Main.c, $(wildcard simulator/*.c))))
BBC_OBJECTS = $(addsuffix .cpp.o, $(basename $(filter-out %Main.cpp, $(wildcard bobbycar-files/*.cpp))))
BBC_COBJECTS = $(addsuffix .c.o, $(basename $(filter-out %Main.c, $(wildcard bobbycar-files/*.c))))
LIBRARIES = -lm

.PRECIOUS: %.o
.SUFFIXES:
.PHONY: all compile

all: compile

compile: $(MAIN_BINARIES) $(CMAIN_BINARIES)

clean:
	rm -f *.o
	rm -f $(MAIN_BINARIES)
	rm -f $(CMAIN_BINARIES)

%Main: %Main.o $(SIM_OBJECTS) $(SIM_COBJECTS) $(BBC_OBJECTS) $(BBC_COBJECTS)
	$(CXX) -o $@ $^ $(LIBRARIES)

%Main.o: %Main.cpp $(CHEADERS) $(HEADERS)
	$(CXX) -c $<

simulator/%.cpp.o: simulator/%.cpp $(CHEADERS) $(HEADERS)
	$(CXX) -c $< -o $<.o

%Main.o: %Main.c $(CHEADERS)
	$(CC) -c $<

simulator/%.c.o: simulator/%.c $(CHEADERS)
	$(CC) -c $< -o $<.o

bobbycar-files/%.cpp.o: bobbycar-files/%.cpp $(CHEADERS) $(HEADERS)
	$(CXX) -c $< -o $<.o

bobbycar-files/%.c.o: bobbycar-files/%.c $(CHEADERS)
	$(CC) -c $< -o $<.o