#
# Makefile for camcalb and posecalb
# 

PROGRAMS = camcalb posecalb invertpose
OBJECTS = camcalb.o ell_markers.o CamPars.o posecalb.o
DEFS =
CINC += -I. -Imatas
CFLAGS += -g -Wall $(CINC) $(DEFS)
LDLIBS += -lcv -lcxcore -lhighgui
LDFLAGS += $(LDLIBS)

all : $(PROGRAMS)

# also generate dependency files
%.o : %.c
	gcc $(CFLAGS) -MM $< > $*.d
	gcc $(CFLAGS) -c $< -o $*.o
%.o : %.cc
	g++ $(CFLAGS) -MM $< > $*.d
	g++ $(CFLAGS) -c $< -o $*.o

camcalb : camcalb.o ell_markers.o CamPars.o
	cd matas; make
	g++ -o $@ camcalb.o ell_markers.o CamPars.o $(LDFLAGS) ./matas/libmatas.a

posecalb: posecalb.o CamPars.o
	g++ -o $@ posecalb.o CamPars.o $(LDFLAGS)

invertpose: invertpose.o
	g++ -o $@ invertpose.o $(LDFLAGS)

tidy:
	rm -f *.o
	rm -f *.d
	rm -f *moc.cc
	cd matas; make tidy

clean: tidy
	rm -f $(PROGRAMS)
	cd matas; make clean

-include $(OBJECTS:.o=.d)

