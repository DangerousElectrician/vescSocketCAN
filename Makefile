INCLUDES = -I.
CXX=g++
CFLAGS=-c -Wall
LDFLAGS= -Wall
SOURCES= $(wildcard *.cpp)
OBJECTS= $(SOURCES:%.cpp=build/%.o)
EXECUTABLE=build/main



.PHONY: directories clean

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@

$(OBJECTS): build/%.o : %.cpp directories
	$(CXX) $(CFLAGS) $(INCLUDES) $< -o $@

directories:
	mkdir -p build

clean:
	rm -r build
