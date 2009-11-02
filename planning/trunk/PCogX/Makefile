# Copyright (c) 2009 Gretton CO.

TARGET=PCogX

CXX ?= g++

CPPFLAGS := -Ipegtl-0.29/include -std=c++0x -pedantic

WARNINGS := -Wall -Wextra

OPTIMISE := -O1

CXXFLAGS := -march=native $(WARNINGS) $(OPTIMISE)

LDFLAGS := -lm -lpthread

SOURCES := $(wildcard *.cc)
DEPENDS := $(SOURCES:.cc=.d)
OBJECTS := $(SOURCES:.cc=.o)

# PHONY lets make know that "all" and "clean" are not file-based
# targets.
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
