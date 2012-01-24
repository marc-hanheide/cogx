#
# Makefile for grabimg, grabstereo
# 

PROGRAMS = grabimg grabstereo grabsequence
DEFS =
CFLAGS += -Wall -O2 $(CINC) $(DEFS) `pkg-config opencv --cflags`
LDLIBS += `pkg-config opencv --libs`
LDFLAGS += $(LDLIBS)

all : $(PROGRAMS)

grabimg : grabimg.c
	g++ -o $@ grabimg.c $(CFLAGS) $(LDFLAGS)

grabstereo : grabstereo.c
	g++ -o $@ grabstereo.c $(CFLAGS) $(LDFLAGS)

grabsequence : grabsequence.c
	g++ -o $@ grabsequence.c $(CFLAGS) $(LDFLAGS)


clean:
	rm -f $(PROGRAMS)

