SRCDIR = src/
INCDIR = include/
OBJDIR = obj/
LIBDIR = lib/

CC = g++
CFLAGS = `pkg-config --cflags opencv sdl` -I$(INCDIR)
LIBS = -lGL -lglut `pkg-config --libs opencv sdl` -L$(LIBDIR) -lTracker
OUT = Tracker

main: standalone.cpp
	make -C BUILD
	make -C BUILD install
	$(CC) standalone.cpp $(CFLAGS) -o $(OUT) $(LIBS)
	
all: clean main

clean:
	sudo rm -f /usr/local/lib/libTracker.so
	make -C BUILD clean
	rm -f *.o $(OUT)
	rm -f *~
	rm -f src/*~
	rm -f include/*~
	rm -f obj/*~
	rm -f resources/*~
	rm -f resources/model/*~
	rm -f resources/shader/*~
	rm -f resources/texture/*~
