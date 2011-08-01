
TARGET=psr

CXX= g++

CXXPREFIX= ccache

OPTIMISE := -O3 -march=native -mno-align-stringops  -minline-all-stringops -ffast-math #-fsee -fsched-spec-load-dangerous -funsafe-loop-optimizations -fprefetch-loop-arrays -ftracer -fipa-matrix-reorg -ftree-parallelize-loops=4  -fmodulo-sched -fmodulo-sched-allow-regmoves -fomit-frame-pointer -finline-functions -funroll-all-loops -mpush-args  -maccumulate-outgoing-args 

WARNINGS := -Wall -Wextra

CPPFLAGS := -ggdb $(OPTIMISE) -pg -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic -Wall


LDFLAGS := #-L/data/private/grettonc/lib/ #-lz  -lm -lpthread #-lglpk -lltdl -lClp -lCoinUtils

CXXFLAGS :=  -march=native $(WARNINGS) $(OPTIMISE)

SOURCES := $(wildcard *.cc)
DEPENDS := $(SOURCES:.cc=.d)
OBJECTS := $(SOURCES:.cc=.o)
LOBJECTS := $(SOURCES:.cc=.lo)

.PHONY: all clean

all: $(DEPENDS) $(OBJECTS) $(TARGET) 

libdtplanning: $(LOBJECTS)
	libtool --mode=link $(CXX) $(LOBJECTS) $(LDFLAGS) -o libpsr.la -module -avoid-version -rpath /usr/local/lib

install: libpsr
	cp libpsr.so /usr/local/lib

%.lo: %.cc
	libtool --mode=compile  $(CXX) $(CPPFLAGS) $< -c -o $@

$(TARGET): $(OBJECTS) net2dtpddl.cpp
	$(CXXPREFIX) $(CXX) -o $@ net2dtpddl.cpp $(OBJECTS) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) 

%.o: %.cc
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) $< -c -o $@

%.d: %.cc
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) -MM -MT $@ $< -o $@

%: %.cc %.d
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(DEPENDS) libpsr.a $(LOBJECTS)
	rm -r .libs

