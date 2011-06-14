
TARGET=pcogx

CXX= g++

CXXPREFIX= 

OPTIMISE := -O3 -march=native -mno-align-stringops  -minline-all-stringops -ffast-math #-fsee -fsched-spec-load-dangerous -funsafe-loop-optimizations -fprefetch-loop-arrays -ftracer -fipa-matrix-reorg -ftree-parallelize-loops=4  -fmodulo-sched -fmodulo-sched-allow-regmoves -fomit-frame-pointer -finline-functions -funroll-all-loops -mpush-args  -maccumulate-outgoing-args 

WARNINGS := -Wall -Wextra

# CPPFLAGS := -ggdb $(WARNINGS) -pg -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic -Wall

# CPPFLAGS :=  $(OPTIMISE)   -DNDEBUG -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic -Wall

#-DLAO_STAR

CPPFLAGS := -ggdb $(OPTIMISE) -DLAO_STAR -DCHARLES_TESTING -pg -ftemplate-depth-1000 -I/data/private/grettonc/include/ -I./pegtl-0.30/include/ -std=c++0x -pedantic -Wall


LDFLAGS := -lm -lpthread  # -L/data/private/grettonc/lib/ -lz  #-lglpk -lltdl -lClp -lCoinUtils

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
	$(CXXPREFIX) $(CXX) -o $@ main.cpp $(OBJECTS) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) 

%.o: %.cc
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) $< -c -o $@

%.d: %.cc
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) -MM -MT $@ $< -o $@

%: %.cc %.d
	$(CXXPREFIX) $(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(DEPENDS) libptplanning.a $(LOBJECTS)
	rm -r .libs
