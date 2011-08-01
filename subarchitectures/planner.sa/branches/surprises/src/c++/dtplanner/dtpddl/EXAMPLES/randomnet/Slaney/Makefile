DEBUGFLAG = -DDEBUG

CC = gcc

DEBUGFLAG = -DDEBUG

# MATHLIB = /opt/SUNWspro/SC2.0.1/libm.a
# MATHLIB = /usr/lib/libm.a
MATHLIB = -lm

CFLAGS = -O3
# CFLAGS = -W -g $(DEBUGFLAG)

all: randomnet

randomnet: randomnet.o
	$(CC) $(CFLAGS) randomnet.o -o randomnet $(MATHLIB)

clean:
	/bin/rm -f *.o core


randomnet.o: randomnet.h
