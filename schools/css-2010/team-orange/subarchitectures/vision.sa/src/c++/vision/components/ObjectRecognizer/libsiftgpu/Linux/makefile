SHELL =/bin/sh
IDIR =../Include
SRC_SIFTGPU = ../SiftGPU/src
SRC_DRIVER = ../TestWin/src
CC=gcc
CFLAGS=  -I$(IDIR) -L/usr/lib -L./bin -fPIC -Wall -Wno-deprecated -pthread  -mfpmath=sse # -march=native 
ODIR_SIFTGPU=siftgpu
ODIR_DRIVER=driver
#external header files
_HEADER_EXTERNAL = Cg/cg.h \
	Cg/cg_bindlocations.h \
	Cg/cg_datatypes.h \
	Cg/cg_enums.h \
	Cg/cg_errors.h \
	Cg/cg_profiles.h \
	Cg/cgGL.h \
	Cg/cgGL_profiles.h \
	GL/glew.h \
	GL/glut.h \
	IL/il.h 
#siftgpu header files
_HEADER_SIFTGPU = 	FrameBufferObject.h \
	GlobalUtil.h \
	GLTexImage.h \
	ProgramCG.h \
	ProgramGPU.h \
	ShaderMan.h \
	ProgramGLSL.h \
	SiftGPU.h \
	SiftPyramid.h \
	PyramidGL.h \
	SiftMatch.h
	
#siftgpu library header files
_HEADER_SIFTGPU_LIB = SiftGPU.h 

#Driver header files	
_HEADER_DRIVER = GLTransform.h 	BasicTestWin.h 	TestWinGlut.h 
	
#the SiftGPU library	
DEPS_SIFTGPU = $(patsubst %,$(IDIR)/%,$(_HEADER_EXTERNAL)) $(pathsubst %, $(SRC_SIFTGPU)/%, $(_HEADER_SIFTGPU))

#Obj files for SiftGPU
_OBJ1 = FrameBufferObject.o GlobalUtil.o GLTexImage.o ProgramCG.o ProgramGLSL.o ProgramGPU.o ShaderMan.o SiftGPU.o SiftPyramid.o PyramidGL.o SiftMatch.o

OBJ_SIFTGPU = $(patsubst %,$(ODIR_SIFTGPU)/%,$(_OBJ1))

#librarys for SiftGPU
LIBS_SIFTGPU = -lCg -lCgGL -lGLEW -lglut -lIL

#rules for 
$(ODIR_SIFTGPU)/%.o: $(SRC_SIFTGPU)/%.cpp $(DEPS_SIFTGPU) 
		$(CC) -o $@ $< $(CFLAGS) -c 

		
#driver dependencies
DEPS_DRIVER =  $(patsubst %,$(IDIR)/%,$(_HEADER_EXTERNAL)) $(pathsubst %, $(SRC_SIFTGPU)/%, $(_HEADER_SIFTGPU_LIB)) $(pathsubst %, $(SRC_DRIVER)/%, $(_HEADER_DRIVER))

#obj files
_OBJ2 = BasicTestWin.o TestWinGlut.o
_OBJ3 = SimpleSIFT.o
_OBJ4 = speed.o

OBJ_TESTWINGLUT = $(patsubst %,$(ODIR_DRIVER)/%,$(_OBJ2))
OBJ_SIMPLESIFT  = $(patsubst %,$(ODIR_DRIVER)/%,$(_OBJ3))	
OBJ_SPEED  = $(patsubst %,$(ODIR_DRIVER)/%,$(_OBJ4))	

#libraries for driver
LIBS_DRIVER=  -lCg -lCgGL -lglut -lIL -lsiftgpu -lGLEW

$(ODIR_DRIVER)/%.o: $(SRC_DRIVER)/%.cpp $(DEPS_DRIVER) 
	$(CC) -o $@ $< $(CFLAGS) -c 

all: makepath $(OBJ_SIFTGPU) $(OBJ_TESTWINGLUT) $(OBJ_SIMPLESIFT) $(OBJ_SPEED)
	ar rcs ./bin/libsiftgpu.a $(OBJ_SIFTGPU) 
	$(CC) -o ./bin/TestWinGlut $(OBJ_TESTWINGLUT)	$(LIBS_DRIVER) $(CFLAGS)
	$(CC) -o ./bin/SimpleSIFT $(OBJ_SIMPLESIFT)	$(LIBS_DRIVER) $(CFLAGS)
	$(CC) -o ./bin/speed $(OBJ_SPEED)	$(LIBS_DRIVER) $(CFLAGS)
	
siftgpu: makepath $(OBJ_SIFTGPU)
	$(CC) -o ./bin/libsiftgpu.so $(OBJ_SIFTGPU)  $(LIBS_SIFTGPU)  $(CFLAGS) -fPIC -shared
	ar rcs ./bin/libsiftgpu.a $(OBJ_SIFTGPU) 
	
driver: makepath $(OBJ_TESTWINGLUT) $(OBJ_SIMPLESIFT) $(OBJ_SPEED)
	$(CC) -o ./bin/TestWinGlut $(OBJ_TESTWINGLUT)	$(LIBS_DRIVER) $(CFLAGS)
	$(CC) -o ./bin/SimpleSIFT $(OBJ_SIMPLESIFT)	$(LIBS_DRIVER) $(CFLAGS)	
	$(CC) -o ./bin/speed $(OBJ_SPEED)	$(LIBS_DRIVER) $(CFLAGS)
makepath:
	-mkdir -p $(ODIR_SIFTGPU)
	-mkdir -p $(ODIR_DRIVER)
	-mkdir -p ./bin	
clean:
	rm -f $(ODIR_SIFTGPU)/*.o
	rm -f $(ODIR_DRIVER)/*.o
	rm -f ./bin/libsiftgpu.a
	rm -f ./bin/libsiftgpu.so
	rm -f ./bin/TestWinGlut
	rm -f ./bin/SimpleSIFT
	rm -f ./bin/speed
