HEADERS = align.h census.h CensusGPU.h
SOURCES = CensusGPU.cpp census.cu
LIB = libgpustereo.so
STATICLIB = libgpustereo.a
PROG = testgpustereo

CC = nvcc
CFLAGS += -I/usr/local/cuda/include -I/usr/local/cuda/sdk/common/inc -I/usr/local/cuda/sdk/C/common/inc
NVCCFLAGS += -Xcompiler -fPIC
LDFLAGS += -L/usr/local/cuda/lib -L/usr/local/cuda/sdk/lib -L/usr/local/cuda/sdk/C/lib
LIBS += -lcudart -lcutil -lhighgui

all : staticlib
lib : $(LIB)
staticlib : $(STATICLIB)
prog : $(PROG)

CensusGPU.o : $(HEADERS) CensusGPU.h
	$(CC) -c CensusGPU.cpp $(CFLAGS) $(NVCCFLAGS)

census.o : $(HEADERS) census.cu
	$(CC) -c census.cu $(CFLAGS) $(NVCCFLAGS)

$(LIB) : CensusGPU.o census.o
	g++ -shared CensusGPU.o census.o $(LDFLAGS) -o $(LIB)

$(STATICLIB) : CensusGPU.o census.o
	ar crs $@ CensusGPU.o census.o

testgpustereo.o : $(HEADERS) testgpustereo.cpp
	$(CC) -c testgpustereo.cpp $(CFLAGS)

$(PROG) : testgpustereo.o $(STATICLIB)
	$(CC) -o $@ testgpustereo.o $(STATICLIB) $(LDFLAGS) $(LIBS)

livegpustereo.o : $(HEADERS) livegpustereo.cpp
	$(CC) -c livegpustereo.cpp $(CFLAGS)

livegpustereo : livegpustereo.o $(STATICLIB)
	$(CC) -o $@ livegpustereo.o $(STATICLIB) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(PROG)
	rm -f $(STATICLIB)
	rm -f $(LIB)
	rm -f CensusGPU.o census.o

