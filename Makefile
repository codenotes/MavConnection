CC=g++
CFLAGS=-c -Wall -std=c++0x -MMD
CONTRIB_DIR=contrib
AR=ar rcs

OBJECTS_DIRECTORY=LibExec
EXECUTABLE_DIRECTORY=TestExec

LIB_NAME=MavCommunication
INC=$(CONTRIB_DIR)
INC_PARAMS=$(foreach d, $(INC), -I$d)

LD_FLAGS=-L./$(OBJECTS_DIRECTORY)
LD_PARAMS=$(foreach d, $(LIB_NAME), -l$d) -lpthread

LIB_SOURCES=$(wildcard src/*.cpp)
LIB_OBJECTS=$(addprefix $(OBJECTS_DIRECTORY)/,$(notdir $(LIB_SOURCES:.cpp=.o)))

TEST_SOURCES=$(wildcard tests/*.cpp) $(wildcard $(CONTRIB_DIR)/gtest/*.cpp)
TEST_OBJECTS=$(addprefix $(OBJECTS_DIRECTORY)/,$(notdir $(TEST_SOURCES:.cpp=.o)))

clean: 
	-rm -f $(OBJECTS_DIRECTORY)/* $(EXECUTABLE_DIRECTORY)/*

$(OBJECTS_DIRECTORY)/%.o: src/%.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) $< -o $@

$(OBJECTS_DIRECTORY)/%.o: tests/%.cpp
	$(CC) $(INC_PARAMS)  $(CFLAGS) $< -o $@

$(OBJECTS_DIRECTORY)/%.o: contrib/gtest/%.cpp
	$(CC) $(INC_PARAMS) $(CFLAGS) $< -o $@

all:  test| mkdir lib

lib: $(LIB_SOURCES) $(LIB_OBJECTS)
	$(AR) $(OBJECTS_DIRECTORY)/lib$(LIB_NAME).a $(LIB_OBJECTS)

test: lib $(TEST_SOURCES) $(TEST_OBJECTS)
	$(CC) $(LD_FLAGS) $(INC_PARAMS)  $(TEST_OBJECTS)  $(LD_PARAMS) -o $(addprefix $(EXECUTABLE_DIRECTORY)/TEST_,$(LIB_NAME))

.PHONY: lib all

mkdir:
	mkdir -p lib
	
#	g++ -std=c++0x -c  -I"contrib" -Wall -o "lib/Hardware.o" "Hardware.h"
#	g++ -std=c++0x -c -I"contrib" -Wall -o "lib/ActionRequestState.o" "ActionRequestState.h"
#	g++ -std=c++0x -c -I"contrib" -Wall -o "lib/MavCommunication.o" "MavCommunication.cpp"
#	g++ -std=c++0x -c -I"contrib" -Wall -o "lib/ProcessSpawner.o" "tests/ProcessSpawner.h"
#	g++ -std=c++0x -c -I"contrib" -Wall -o "lib/AllTests" "tests/AllTests.cpp"
