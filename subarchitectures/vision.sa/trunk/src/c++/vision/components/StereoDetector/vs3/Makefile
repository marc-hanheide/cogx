#
# $Id: Makefile,v 1.44 2007/07/27 17:01:25 mxz Exp mxz $
# 
# Makefile for vs3
# 

PROGRAM = vs3
LIB = libvs3.a
OBJECTS = VisionCore.o Segment.o Math.o ConfigFile.o\
 BufferVideo.o OpenCvImgSeqVideo.o Config.o Vector2.o\
 Except.o Color.o Gestalt.o GestaltPrinciple.o\
 IdImage.o FormSegments.o\
 Line.o FormLines.o\
 Closure.o\
 TJunction.o FormJunctions.o LJunction.o Collinearity.o\
 Rectangle.o nrutil.o\
 VoteImage.o\
 betai.o gammln.o betacf.o OpenCvLiveVideo.o rosin_lines.o Arc.o AJunction.o\
 fitcircle.o FormArcJunctions.o FormArcs.o\
 Ellipse.o ConvexArcGroup.o FormConvexArcGroups.o FormEllipses.o \
 FormClosures.o Flap.o FormFlaps.o
# rosin_arcline.o fitellipse.o fitcircle.o FormRectangles.o ParallelLineGroup.o
QTOBJECTS = Main.o MainWin.o MainWin_moc.o Draw.o
NOGUIOBJECTS = main_small.o Draw_small.o

# Compiler flags 
DEFS = -DUSE_OPENCV_ELLFIT -DUSE_COLOR
# -DUSE_OPENCV_ELLFIT -DDEBUG_CHECKINDEX -DUSE_COLOR
CINC = -I. -Imatas `pkg-config QtGui QtCore QtOpenGL Qt3Support --cflags`
CFLAGS = -Wall -O2 $(CINC) $(DEFS)
LDLIBS = ./matas/libmatas.a -lQtGui -lGLU -lglut -lcv -lcxcore -lhighgui\
 -lpthread -ldl -lm -lrt
# -lSegFault
LDFLAGS = -L/usr/X11/lib `pkg-config QtGui QtCore QtOpenGL Qt3Support --libs` $(LDLIBS)

all : $(PROGRAM)
lib : $(LIB)

MainWin_moc.cc : MainWin.hh
	moc $< > $@

# also generate dependency files
%.o : %.c
	gcc $(CFLAGS) -MM $< > $*.d
	gcc $(CFLAGS) -c $< -o $*.o
%.o : %.cc
	g++ $(CFLAGS) -MM $< > $*.d
	g++ $(CFLAGS) -c $< -o $*.o

$(LIB) : $(OBJECTS)
	cd matas; make
	ar crs $(LIB) $(OBJECTS)

$(PROGRAM) : $(OBJECTS) $(QTOBJECTS)
	cd matas; make
	g++ -o $@ $(OBJECTS) $(QTOBJECTS) $(LDFLAGS)

vs3nogui : $(OBJECTS) $(NOGUIOBJECTS)
	cd matas; make
	g++ -o $@ $(OBJECTS) $(NOGUIOBJECTS) $(LDFLAGS)

tidy:
	rm -f *.o
	rm -f *.d
	rm -f *moc.cc
	cd matas; make tidy

clean: tidy
	rm -f $(PROGRAM)
	rm -f $(LIB)
	cd matas; make clean

-include $(OBJECTS:.o=.d)
-include $(QTOBJECTS:.o=.d)
-include $(ELLOBJECTS:.o=.d)

