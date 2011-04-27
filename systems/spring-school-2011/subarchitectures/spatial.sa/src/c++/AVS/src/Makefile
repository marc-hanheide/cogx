CC = g++
CFLAGS = -shared -fPIC
LIBSDIR = -L /home/alper/projects/cosycure/lib/cure/
LIBS = -lcuretoolbox -lpthread
INCLUDEDIR = -I /home/alper/projects/cosycure/src/
COGXLIBDIR = /home/cogx/projects/kthsys/output/lib
LIBRARY = libAVSPolicyManager.so
SOURCES =  AVSPolicyManager.cc AVSPolicyManager.hh

TEST = Test.cc

$(LIBRARY) : $(SOURCES)
	$(CC) $(CFLAGS)  $(LIBSDIR) $(LIBS) $(SOURCES) $(INCLUDEDIR) -o $(LIBRARY)

clean : 
	 rm -f *.so; rm -f *.out; rm -f *.o 

test : $(TEST)
	$(CC) $(LIBSDIR) -L ./ $(LIBS) -lAVSPolicyManager $(TEST)  $(INCLUDEDIR)

all : $(LIBRARY) test
