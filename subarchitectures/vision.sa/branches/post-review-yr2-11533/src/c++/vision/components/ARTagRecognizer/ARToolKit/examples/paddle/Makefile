INC_DIR= ../../include
LIB_DIR= ../../lib
BIN_DIR= ../../bin

LDFLAG=-L/usr/X11R6/lib -L$(LIB_DIR)
LIBS= -lARgsub -lARvideo -lARMulti -lAR -lglut -lGLU -lGL -lXi -lXmu -lX11 -lm -ljpeg
CFLAG= -O -I/usr/X11R6/include -DUSE_EYETOY -I/usr/src/linux-headers-2.6.38.2-hpusb/include -I/usr/src/linux-headers-2.6.38.2-hpusb/arch/x86/include -I$(INC_DIR)

OBJS = paddle.o
HEADDERS = paddle.h

all: $(BIN_DIR)/paddleTest

$(BIN_DIR)/paddleTest: paddleTest.o $(OBJS)
	cc -o $(BIN_DIR)/paddleTest paddleTest.o $(OBJS) $(LDFLAG) $(LIBS)

paddleTest.o: paddleTest.c $(HEADDERS)
	cc -c $(CFLAG) paddleTest.c

paddle.o: paddle.c $(HEADDERS)
	cc -c $(CFLAG) paddle.c

clean:
	rm -f *.o
	rm -f $(BIN_DIR)/paddleTest

allclean:
	rm -f *.o
	rm -f $(BIN_DIR)/paddleTest
	rm -f Makefile
