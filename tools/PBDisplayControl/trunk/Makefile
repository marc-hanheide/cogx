NAME=PBDisplayControl
CFLAGS=-g -Wall -o $(NAME) -lboost_program_options
GTKFLAGS=`pkg-config --cflags --libs gtkmm-2.4 peekabot gthread-2.0`
SRCS=PBDisplayControl.cpp
CC=g++
 
# Do all
all: main
 
# Compile
main: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) $(GTKFLAGS)
 
# Clean
clean:
	rm -f $(NAME)
	rm -f *.h~
	rm -f *.cc~
	rm -f Makefile~
	rm -f *.glade~
