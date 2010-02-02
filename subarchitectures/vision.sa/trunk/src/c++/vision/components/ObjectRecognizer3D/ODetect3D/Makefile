#
# $Id$
# 

PROGRAM = odetect 
LIB = libODetect.a
OBJECTS = ODetect3D.o CodebookEntry.o Object3D.o Pose.o ModelObject3D.o\
 Keypoint.o KeypointDescriptor.o DetectGPUSIFT.o DetectSIFT.o\
 Vector2.o SDraw.o SPolygon.o Homography.o Geometry.o Except.o Math.o Config.o ConfigFile.o\
 PoseCv.o Vector3.o

GUIOBJECTS = Main.o

DEFS =
CINC = -I. -I../matrix -I/home/hannes/software/SiftGPU/SiftGPU/src
CFLAGS = -Wall -g $(CINC) $(DEFS)
LDLIBS = -lmatrix\
 -lIL -lGL -lglut -lGLEW -lCgGL -lCg -lsiftgpu\
 -lGLU -lcv -lcxcore -lhighgui -lpthread -ldl -lm -lrt
LDFLAGS = -L/home/hannes/software/SiftGPU/Linux/bin\
 -L../matrix $(LDLIBS)

all : $(PROGRAM)
lib : $(LIB)

# also generate dependency files
%.o : %.c
	gcc $(CFLAGS) -MM $< > $*.d
	gcc $(CFLAGS) -c $< -o $*.o
%.o : %.cc
	g++ $(CFLAGS) -MM $< > $*.d
	g++ $(CFLAGS) -c $< -o $*.o

$(LIB) : $(OBJECTS)
	ar crs $(LIB) $(OBJECTS)

$(PROGRAM) : $(OBJECTS) $(GUIOBJECTS) 
	g++ -o $@ $(OBJECTS) $(GUIOBJECTS) $(LDFLAGS)

tidy:
	rm -f *.o
	rm -f *.d

clean: tidy
	rm -f $(PROGRAM)
	rm -f $(LIB)

-include $(OBJECTS:.o=.d)

