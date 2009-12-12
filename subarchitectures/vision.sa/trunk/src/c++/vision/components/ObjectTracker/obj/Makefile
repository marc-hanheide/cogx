SRCDIR = ../src/
INCDIR = ../include/

SRC = $(SRCDIR)*.cpp
CC = g++

CFLAGS = `pkg-config --cflags opencv sdl` -DGL_GLEXT_PROTOTYPES 
LIBS = -lglut -lrt
LIB_SDL = -lSDL -lSDLmain -lSDL_image

all: *.o ply.o

*.o: $(SRCDIR)*.cpp
	$(CC) $(CFLAGS) -c $(SRCDIR)*.cpp -I$(INCDIR)

ply.o: $(SRCDIR)ply.c
	cc -c -I$(INCDIR) $(SRCDIR)ply.c
	
clean:
	rm -f *.o
