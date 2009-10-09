#
# Makefile for genroi
# 

PROGRAMS = genroi
DEFS =
CFLAGS += -Wall -O2 $(CINC) $(DEFS) `pkg-config opencv --cflags`
LDLIBS += `pkg-config opencv --libs`
LDFLAGS += $(LDLIBS)

all : $(PROGRAMS)

genroi: genroi.cc
	g++ -o $@ genroi.cc $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(PROGRAMS)

