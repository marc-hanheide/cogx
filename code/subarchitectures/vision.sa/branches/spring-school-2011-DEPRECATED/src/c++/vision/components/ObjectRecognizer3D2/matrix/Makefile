#
# Makefile for matrix library
# 

LIB = libmatrix.a
OBJECTS = alloc.o \
  choleski.o \
  construc.o \
  creat.o \
  inverse.o \
  io.o \
  is.o \
  jacobi.o \
  jordan.o \
  lu.o \
  manip.o \
  norm.o \
  operator.o \
  points.o \
  ql.o \
  qr.o \
  quatern.o \
  rotation.o \
  svdcmp.o \
  system.o \
  unary.o \
  vector.o
 
# Compiler flags 
CINC = -I. 
CFLAGS = -Wall -O2 $(CINC)

all : $(LIB)

# also generate dependency files
%.o : %.c
	gcc $(CFLAGS) -MM $< > $*.d
	gcc $(CFLAGS) -c $< -o $*.o

$(LIB) : $(OBJECTS)
	ar crs $@ $(OBJECTS)

tidy:
	rm -f *.o
	rm -f *.d

clean: tidy
	rm -f $(LIB)

-include $(OBJECTS:.o=.d)

