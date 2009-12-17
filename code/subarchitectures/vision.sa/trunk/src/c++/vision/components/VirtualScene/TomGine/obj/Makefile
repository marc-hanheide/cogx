SRCDIR = ../src/
INCDIR = ../include/

SRC = $(SRCDIR)*.cpp
CC = g++

CFLAGS = `pkg-config --cflags opencv sdl` -DGL_GLEXT_PROTOTYPES 
LIBS = -lglut -lrt
LIB_SDL = -lSDL -lSDLmain -lSDL_image

all: *.o

*.o: $(SRCDIR)*.cpp
	$(CC) $(CFLAGS) -c $(SRCDIR)*.cpp -I$(INCDIR)

clean:
	rm -f *.o
