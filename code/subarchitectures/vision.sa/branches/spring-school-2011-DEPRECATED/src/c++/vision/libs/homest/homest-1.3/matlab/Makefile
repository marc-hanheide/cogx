#
# Unix/Linux Makefile Makefile for MATLAB interface to homest
#

MEX=mex
MEXCFLAGS=-I.. -O #-g
# WHEN USING LAPACK, CHANGE THE NEXT TWO LINES TO WHERE YOUR COMPILED LAPACK/BLAS & F2C LIBS ARE!
LAPACKBLASLIBS_PATH=/usr/lib
F2CLIBS_PATH=/usr/local/lib
LEVMARLIBSPATH=/home/lourakis/levmar-src/levmar-2.4

INTFACESRCS=homest.c
INTFACESOBJ=homest.mexglx
LEVMARLIB=$(LEVMARLIBSPATH)/liblevmar.a
HOMESTLIB=../libhomest.a
LAPACKLIBS=$(LAPACKBLASLIBS_PATH)/liblapack.a $(LAPACKBLASLIBS_PATH)/libblas.a $(F2CLIBS_PATH)/libf2c.a

LIBS=$(HOMESTLIB) $(LEVMARLIB) $(LAPACKLIBS)

dummy: $(INTFACESOBJ)

$(INTFACESOBJ): $(INTFACESRCS) $(HOMESTLIB)
	$(MEX) $(MEXCFLAGS) $(INTFACESRCS) $(LIBS)

clean:
	@rm -f $(INTFACESOBJ)

realclean cleanall: clean

depend:
	makedepend -f Makefile $(INTFACESRCS)

# DO NOT DELETE THIS LINE -- make depend depends on it.

