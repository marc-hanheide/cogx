SRCDIR = src/
INCDIR = include/
OBJDIR = obj/

CC = g++
CFLAGS = `pkg-config --cflags opencv sdl` -I$(INCDIR)
LIBS = -lGL -lglut `pkg-config --libs opencv sdl`
OUT = main

main: *.cpp *.o
	$(CC) *.cpp $(CFLAGS) -o $(OUT) $(LIBS) $(OBJDIR)*.o

*.o:
	make -C $(OBJDIR)
	
new: clean main

clean:
	rm -f *.o main
	make -C $(OBJDIR) clean

