INC_DIR= ../../include
LIB_DIR= ../../lib
BIN_DIR= ../../bin

LDFLAG=-L/usr/X11R6/lib -L$(LIB_DIR)
LIBS= -lARgsub_lite -lARvideo -lAR -lglut -lGLU -lGL -lXi -lXmu -lX11 -lm -ljpeg
CFLAG= -O -I/usr/X11R6/include -DUSE_EYETOY -I/usr/src/linux-headers-2.6.38.2-hpusb/include -I/usr/src/linux-headers-2.6.38.2-hpusb/arch/x86/include -I$(INC_DIR)

OBJS =
HEADDERS =

all: $(BIN_DIR)/twoView

$(BIN_DIR)/twoView: twoView.o $(OBJS)
	cc -o $(BIN_DIR)/twoView twoView.o $(OBJS) $(LDFLAG) $(LIBS)

twoView.o: twoView.c $(HEADDERS)
	cc -c $(CFLAG) twoView.c

clean:
	rm -f *.o
	rm -f $(BIN_DIR)/twoView

allclean:
	rm -f *.o
	rm -f $(BIN_DIR)/twoView
	rm -f Makefile
