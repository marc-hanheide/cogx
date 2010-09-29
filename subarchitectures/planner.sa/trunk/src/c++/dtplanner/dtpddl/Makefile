
TARGET=some

CXX= g++

CXXPREFIX= #ccache

OPTIMISE := -O3 -march=native 

WARNINGS := -Wall -Wextra

CPPFLAGS := -ggdb $(WARNINGS) -pg -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic

# CPPFLAGS := $(OPTIMISE) -DNDEBUG -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic
 
# CPPFLAGS := -ggdb $(OPTIMISE) -pg -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic


LDFLAGS := -L/data/private/grettonc/lib/ -lz  -lm -lpthread #-lglpk -lltdl -lClp -lCoinUtils

CXXFLAGS :=  -march=native $(WARNINGS) $(OPTIMISE)

SOURCES := $(wildcard *.cc)
DEPENDS := $(SOURCES:.cc=.d)
OBJECTS := $(SOURCES:.cc=.o)
LOBJECTS := $(SOURCES:.cc=.lo)

.PHONY: all clean

all: $(DEPENDS) $(OBJECTS) $(TARGET) 

libdtplanning: $(LOBJECTS)
	libtool --mode=link $(CXX) $(LOBJECTS) $(LDFLAGS) -o libdtplanning.la -module -avoid-version -rpath /usr/local/lib

install: libdtplanning
	cp libdtplanning.so /usr/local/lib

%.lo: %.cc
	libtool --mode=compile  $(CXX) $(CPPFLAGS) $< -c -o $@

$(TARGET): $(OBJECTS) main.cpp
	$(CXX) -o $@ main.cpp $(OBJECTS) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) 

%.o: %.cc
	$(CXX) $(CPPFLAGS) $< -c -o $@

%.d: %.cc
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) -MM -MT $@ $< -o $@

%: %.cc %.d
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(DEPENDS) libptplanning.a $(LOBJECTS)
	rm -r .libs
