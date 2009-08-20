#
# Makefile for genroi
# 

PROGRAMS = genroi
DEFS =
CFLAGS += -Wall -O2 $(CINC) $(DEFS)
LDLIBS += -lcv -lhighgui
LDFLAGS += $(LDLIBS)

all : $(PROGRAMS)

genroi: genroi.cc
	g++ -o $@ genroi.cc $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(PROGRAMS)

