# SLICE

SLICE_PATH?=../../slice
SLICE_FILES=asr.ice asr-loquendo.ice

SLICE_LIB_INCLUDE=$(ICE_HOME)/share/Ice/slice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# C++

EXEC?=testrecord

CXX_HEADERS=
CXX_SOURCES=MainRecordPCM.cc

CXXFLAGS+=-Wall
CXXFLAGS+=-O2
CXXFLAGS+=-g
CXXFLAGS+=-I../utils
CXXFLAGS+=-I../audiosource
CXXFLAGS+=-I$(LASR_INSTALL_PATH)includes
CXXFLAGS+=-DPROGRAM_NAME=\"$(EXEC)\"

LDFLAGS+=-L../utils -ltty
LDFLAGS+=-L../audiosource -laudiosource
LDFLAGS+=-lpulse-simple
LDFLAGS+=-L$(LASR_INSTALL_PATH)bin -lLoqASR

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: main
main: $(EXEC)

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.
#------------------------------------------------------------------------------#

ifeq ($(origin DEBUG), undefined)
else
CXXFLAGS+=-DDEBUG
endif

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

OBJS=$(addsuffix .o, $(basename $(CXX_SOURCES)))

%.o: %.cc $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

$(EXEC): $(CXX_HEADERS) $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	rm -rf $(OBJS) \
		$(EXEC)
