SRCDIR = src/
INCDIR = include/
OBJDIR = obj/

CC = g++
CFLAGS = `pkg-config --cflags opencv sdl` -I$(INCDIR)
LIBS = -lGL -lglut `pkg-config --libs opencv sdl`
OUT = tracker

main: standalone.cpp *.o
	$(CC) standalone.cpp $(CFLAGS) -o $(OUT) $(LIBS) $(OBJDIR)*.o

*.o:
	make -C $(OBJDIR)
	
all: clean main

clean:
	rm -f *.o $(OUT)
	make -C $(OBJDIR) clean
	rm -f *~
	rm -f src/*~
	rm -f include/*~
	rm -f obj/*~
	rm -f resources/*~
	rm -f resources/model/*~
	rm -f resources/shader/*~
	rm -f resources/texture/*~
