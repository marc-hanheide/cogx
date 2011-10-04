#------------------------------------------------------------------------------#

# Slice

SLICE_PATH?=../../slice
SLICE_FILES=asr-loquendo.ice

SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/share/Ice/slice
SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/share/slice
SLICE_LIB_INCLUDE+=-I$(ICE_HOME)/slice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# C++

EXEC?=lasrctrl

CXX_HEADERS=CLI.h
CXX_SOURCES=MainControl.cc CLI.cc

CXXFLAGS+=-Wall
CXXFLAGS+=-O2
CXXFLAGS+=-I. $(ICE_INCLUDE) -I../utils
CXXFLAGS+=-g
CXXFLAGS+=-DPROGRAM_NAME=\"$(EXEC)\"

LDFLAGS+=-L../utils
LDFLAGS+=$(ICE_LINK) -lIce -lIceUtil

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

.PHONY: main
main: $(EXEC)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# TODO: how to generate this automatically?

SLICE_CXX_HEADERS=$(addsuffix .h, $(basename $(SLICE_FILES)))
SLICE_CXX_SOURCES=$(addsuffix .cc, $(basename $(SLICE_FILES)))

asr-loquendo.h asr-loquendo.cc: $(SLICE_PATH)/asr-loquendo.ice
	slice2cpp -I$(SLICE_PATH) $(SLICE_LIB_INCLUDE) --source-ext cc $(SLICE_PATH)/asr-loquendo.ice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

OBJS=$(addsuffix .o, $(basename $(SLICE_CXX_SOURCES) $(CXX_SOURCES)))

%.o: %.cc $(SLICE_CXX_HEADERS) $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

$(EXEC): $(SLICE_CXX_HEADERS) $(CXX_HEADERS) $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	rm -rf $(OBJS) \
		$(SLICE_CXX_HEADERS) $(SLICE_CXX_SOURCES) \
		$(EXEC)
