#
# Makefile for matas library, used in our case for canny edge detection
# 

LIB = libmatas.a
OBJECTS = ConstSet.o gradient.o LL.o graph.o quantAry.o readAry.o LLconsis.o\
canny_body.o scaleFary.o LLfile.o colorquant.o LLio.o initAry.o LLmergeSort.o\
consP.o entityP.o LLstr.o consVectsP.o errorHand.o makary.o PGerror.o\
convertAry.o normFary.o thrLink.o PGio.o copyAry.o vector2.o PGutil.o\
destSets.o extendAry.o RWSets.o formatP.o option.o vector2i.o StdGeomDefP.o\
getSetP.o writeAry.o optionDouble.o writeGF.o optionInt.o xyEdge2LL.o ary.o\
optionList.o optionMisc.o arg2Str.o buffStr.o consStr.o dupStr.o strcasecmp.o\
suffStr.o tokenStr.o tokenPref.o 

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

