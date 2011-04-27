
TARGET=some

CXX=  g++

OPTIMISE := -O1 

CPPFLAGS := -ggdb  -I./pegtl-0.29/include/ -I/data/private/grettonc/include/ -I/data/private/grettonc/include/coin/ -std=c++0x -pedantic

WARNINGS := -Wall -Wextra

LDFLAGS := -L/data/private/grettonc/lib/ -lglpk -lz -lltdl -lClp -lCoinUtils -lm -lpthread

CXXFLAGS :=  -march=native $(WARNINGS) $(OPTIMISE)

SOURCES := $(wildcard *.cc)
DEPENDS := $(SOURCES:.cc=.d)
OBJECTS := $(SOURCES:.cc=.o)

.PHONY: all clean

all: $(DEPENDS) $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS) 
	$(CXX) -o $@ $(OBJECTS) $(CXXFLAGS) $(LDFLAGS)

%.o: %.cc
	$(CXX) $(CPPFLAGS) $< -c -o $@

%.d: %.cc
	$(CXX) $(CPPFLAGS) -MM -MT $@ $< -o $@

%: %.cc %.d
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(DEPENDS)
