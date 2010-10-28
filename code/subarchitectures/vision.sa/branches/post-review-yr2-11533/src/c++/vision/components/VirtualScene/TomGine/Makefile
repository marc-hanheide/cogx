SRCDIR = src/
INCDIR = include/
LIBDIR = -Llib/

CC = g++
CFLAGS = `pkg-config --cflags opencv sdl freetype2` -I$(INCDIR)
LIBS = $(LIBDIR) -lGL -lglut `pkg-config --libs opencv sdl ftgl` -lTomGine
OUT = TomGine

main: *.cpp
	$(CC) *.cpp $(CFLAGS) -o $(OUT) $(LIBS)
	
all: clean main

clean:
	sudo rm -f /usr/local/lib/libTomGine.so
	make -C BUILD clean
	rm -f *.o $(OUT)
	rm -f *~
	rm -f src/*~
	rm -f include/*~
	rm -f resources/*~
	rm -f resources/model/*~
	rm -f resources/shader/*~
	rm -f resources/texture/*~

