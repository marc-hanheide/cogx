
TARGET=some

CXX= ccache g++

OPTIMISE := -O1 

CPPFLAGS := -ggdb -ftemplate-depth-1000 -I./pegtl-0.30/include/ -I/data/private/grettonc/include/ -I/data/private/grettonc/include/coin/ -std=c++0x -pedantic

WARNINGS := -Wall -Wextra

LDFLAGS := -L/data/private/grettonc/lib/ -lz  -lm -lpthread #-lglpk -lltdl -lClp -lCoinUtils

CXXFLAGS :=  -march=native $(WARNINGS) $(OPTIMISE)

SOURCES := $(wildcard *.cc)
DEPENDS := $(SOURCES:.cc=.d)
OBJECTS := $(SOURCES:.cc=.o)

.PHONY: all clean

all: $(DEPENDS) $(OBJECTS) $(TARGET)

libdtplanning: $(OBJECTS)
	ar rv libptplanning.a $(OBJECTS)
	ranlib libptplanning.a


$(TARGET): $(OBJECTS) 
	$(CXX) -o $@ main.cpp $(OBJECTS) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) 

%.o: %.cc
	$(CXX) $(CPPFLAGS) $< -c -o $@

%.d: %.cc
	$(CXX) $(CPPFLAGS) -MM -MT $@ $< -o $@

%: %.cc %.d
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(DEPENDS) libptplanning.a
