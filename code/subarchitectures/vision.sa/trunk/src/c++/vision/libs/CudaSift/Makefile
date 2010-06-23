# Settings
SOURCEDIR   ?= .
DESTDIR 	?= obj
PYVERSION 	?= 2.6
PYINCLUDE   ?= -I/usr/include/python$(PYVERSION)
PYLIBDIR    ?= -L/usr/lib/python$(PYVERSION)/config
PYLINK      ?= -lpython$(PYVERSION)
LIBCUTIL    ?= # /usr/local/cuda/sdk/lib/libcutil.a

ifeq ($(strip $(SOURCEDIR)),)
  SRCDIR=.
else
  SRCDIR=$(SOURCEDIR)
endif
ifeq ($(strip $(DESTDIR)),)
  OBJDIR=.
else
  OBJDIR=$(DESTDIR)
endif
	
# Input files
IHEADERS = \
	cudaImage.h \
	cudaSiftD.h \
	cudaSift.h \
	cudaSiftH.h \
	tpimage.h \
	tpimageutil.h \
	featureUploadH.h 
ISOURCES = \
	cudaImage.cu \
	cudaSiftH.cu \
	featureUploadH.cu \
	pysiftcuda.cpp 
ICU_DEPS = cudaSiftD.cu 
LIB = siftcuda.so

# Full pathnames
HEADERS=$(addprefix $(SRCDIR)/,$(IHEADERS))
SOURCES=$(addprefix $(SRCDIR)/,$(ISOURCES))
CU_DEPS=$(addprefix $(SRCDIR)/,$(ICU_DEPS))
LIBTARGET=$(OBJDIR)/$(LIB)

CC = /usr/local/cuda/bin/nvcc
CFLAGS += -I/usr/local/cuda/include -I/usr/local/cuda/sdk/common/inc $(PYINCLUDE)
CFLAGS += -I/usr/local/cuda/C/include -I/usr/local/cuda/sdk/C/common/inc
NVCCFLAGS += -Xcompiler -fPIC,-fno-strict-aliasing,-DUNIX
# NVCCFLAGS += -Xcompiler -O2 # (ubuntu 10.04) --> error with 'weak inline functions'
LDFLAGS += -fPIC -L/usr/local/cuda/lib -L/usr/local/cuda/sdk/lib $(PYLIBDIR)
LDFLAGS += -L/usr/local/cuda/sdk/C/lib
LIBS += $(PYLINK) -lcudart # -lcutil 

all : lib
lib : $(LIBTARGET)

$(OBJDIR)/pysiftcuda.o : $(SRCDIR)/pysiftcuda.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(NVCCFLAGS) -o $@ -c $(SRCDIR)/pysiftcuda.cpp
	
$(OBJDIR)/cudaImage.o : $(SRCDIR)/cudaImage.cu $(HEADERS) $(CU_DEPS)
	$(CC) $(CFLAGS) $(NVCCFLAGS) -o $@ -c $(SRCDIR)/cudaImage.cu

$(OBJDIR)/cudaSiftH.o : $(SRCDIR)/cudaSiftH.cu $(HEADERS) $(CU_DEPS)
	$(CC) $(CFLAGS) $(NVCCFLAGS) -o $@ -c $(SRCDIR)/cudaSiftH.cu

$(OBJDIR)/featureUploadH.o : $(SRCDIR)/featureUploadH.cu $(HEADERS) $(CU_DEPS)
	$(CC) $(CFLAGS) $(NVCCFLAGS) -o $@ -c $(SRCDIR)/featureUploadH.cu

OBJS= \
	$(OBJDIR)/pysiftcuda.o \
	$(OBJDIR)/cudaImage.o \
	$(OBJDIR)/cudaSiftH.o \
	$(OBJDIR)/featureUploadH.o
	# $(OBJDIR)/cudaSiftD.o \

$(LIBTARGET) : directories $(OBJS)
	g++ -shared $(LDFLAGS) $(LIBS) -o $(LIBTARGET) $(OBJS) $(LIBCUTIL)

directories: $(OBJDIR)
$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -f $(LIBTARGET)
	rm -f $(OBJS)

# include Makefile.dep
