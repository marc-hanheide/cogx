#------------------------------------------------------------------------------#

# SLICE

SLICE_PATH?=../../slice
SLICE_FILES=asr-loquendo.ice

SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/share/Ice/slice
SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/share/slice
SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/slice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# C++

EXEC?=lasr-ice

CXX_HEADERS=Version.h LoquendoSession.h LoquendoInstance.h InstanceException.h SessionException.h LoquendoRecogniserServer.h CLI.h
CXX_HEADERS+=RecognitionThread.h
CXX_HEADERS+=SLFReader.h

CXX_SOURCES=LoquendoSession.cc LoquendoInstance.cc InstanceException.cc SessionException.cc LoquendoRecogniserServer.cc Main.cc CLI.cc
CXX_SOURCES+=RecognitionThread.cc
CXX_SOURCES+=SLFReader.cc

CXXFLAGS+=-Wall
CXXFLAGS+=-O2
CXXFLAGS+=-g
CXXFLAGS+=-I../utils
CXXFLAGS+=-I../audiosource
CXXFLAGS+=-I. $(ICE_INCLUDE)
CXXFLAGS+=-I$(LASR_INSTALL_PATH)/includes
CXXFLAGS+=$(shell pkg-config --cflags libpulse-simple)
CXXFLAGS+=$(shell pkg-config --cflags liblog4cxx)
CXXFLAGS+=-pthread
CXXFLAGS+=-DPROGRAM_NAME=\"$(EXEC)\"

LDFLAGS+=-L../audiosource -laudiosource
LDFLAGS+=$(ICE_LINK) -lIce -lIceUtil -pthread
LDFLAGS+=$(shell pkg-config --libs libpulse-simple)
LDFLAGS+=$(shell pkg-config --libs liblog4cxx)
LDFLAGS+=-L$(LASR_INSTALL_PATH)/bin -lLoqASR -Wl,-rpath $(LASR_INSTALL_PATH)/bin

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: main
main: $(EXEC)

.PHONY: test
test: test-slfreader

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.

ifeq ($(origin ICE_HOME), undefined)
else
ICE_LINK=-L$(ICE_HOME)/lib
ICE_INCLUDE=-I$(ICE_HOME)/include
endif

# on BSD, without deactivating suffix rules, any "non-suffixed"
# target would be matched by some sort of default rule (using
# $(CC)), not by our rule.
.SUFFIXES:

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

ifeq ($(origin DEBUG), undefined)
else
CXXFLAGS+=-DDEBUG
endif

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# TODO: how to generate this automatically?

SLICE_CXX_HEADERS=$(addsuffix .h, $(basename $(SLICE_FILES)))
SLICE_CXX_SOURCES=$(addsuffix .cc, $(basename $(SLICE_FILES)))

asr-loquendo.h asr-loquendo.cc: $(SLICE_PATH)/asr-loquendo.ice
	slice2cpp -I$(SLICE_PATH) $(SLICE_LIB_INCLUDE) --source-ext cc $(SLICE_PATH)/asr-loquendo.ice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

SLICE_OBJS=$(addsuffix .o, $(basename $(SLICE_CXX_SOURCES)))
OBJS=$(SLICE_OBJS) $(addsuffix .o, $(basename $(CXX_SOURCES)))

%.o: %.cc $(SLICE_CXX_HEADERS) $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

$(EXEC): $(SLICE_CXX_HEADERS) $(CXX_HEADERS) $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

test-slfreader: SLFReaderTest.o SLFReader.o $(SLICE_OBJS)
	$(CXX) SLFReaderTest.o SLFReader.o $(SLICE_OBJS) -o test-slfreader $(LDFLAGS)

#------------------------------------------------------------------------------#

.PHONY: clean
clean:
	rm -rf $(OBJS) SLFReaderTest.o \
		$(addsuffix .cc, $(basename $(SLICE_FILES))) $(addsuffix .h, $(basename $(SLICE_FILES))) \
		$(EXEC) test-slfreader
