#
# Unix/Linux GCC Makefile for homography estimation
#

CC=gcc
LEVMAR_PATH=/home/lourakis/levmar-src/levmar-2.4/ # CHANGE THIS TO POINT TO YOUR COMPILED COPY OF LEVMAR
INCLUDES=-I$(LEVMAR_PATH)
CFLAGS=$(INCLUDES) -O3 -funroll-loops -Wall #-pg
LAPACKLIBS_PATH=/usr/local/lib # WHEN USING LAPACK, CHANGE THIS TO WHERE YOUR COMPILED LIBS ARE!
LDFLAGS=-L. -L$(LEVMAR_PATH) -L$(LAPACKLIBS_PATH)

LIBOBJS=calc_2Dhomog_coeffs.o homest.o lqs.o linalg.o norm.o buckets.o
LIBSRCS=calc_2Dhomog_coeffs.c homest.c lqs.c linalg.c norm.c buckets.c

DEMOOBJS=homest_demo.o
DEMOSRCS=homest_demo.c
AR=ar
RANLIB=ranlib
RM=rm -f
MAKE=make
MAPLE=maple

LAPACKLIBS=-llapack -lblas #-lf2c

all: libhomest.a homest_demo

libhomest.a: $(LIBOBJS)
	$(AR) crv libhomest.a $(LIBOBJS)
	$(RANLIB) libhomest.a

calc_2Dhomog_coeffs.o: maplefuncs.h
homest.o: util.h homest.h maplefuncs.h lqs.h
lqs.o: lqs.h compiler.h
linalg.o: compiler.h util.h
norm.o: compiler.h util.h
buckets.o: util.h lqs.h
    
homest_demo: $(DEMOOBJS) libhomest.a
	$(CC) $(LDFLAGS) $(DEMOOBJS) -o homest_demo -lhomest -llevmar $(LAPACKLIBS) -lm 

homest_demo.o: homest.h

#calc_2Dhomog_coeffs.c: homo2D.mpl
#	$(MAPLE) <  $<

clean:
	-$(RM) $(LIBOBJS) $(DEMOOBJS) gmon.out

cleanall: clean
	-$(RM) libhomest.a

depend:
	makedepend $(INCLUDES) -f Makefile $(LIBSRCS) $(DEMOSRCS)

# DO NOT DELETE THIS LINE -- make depend depends on it.

