CXXFLAGS=-Wall
LDFLAGS=-L$(ICE_HOME)/lib -lIce -lIceUtil

BIN=abducer-server-mk2

HEADERS=Tokens.h TermTokeniser.h TermParsing.h Terms.h Abducer.h

CXX_SOURCES=Tokens.cc TermTokeniser.cc ServerMain.cc TermParsing.cc Terms.cc Abducer.cc

#------------------------------------------------------------------------------#

SLICE_PATH?=../../../slice

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.

OBJS=$(addsuffix .o, $(basename $(CXX_SOURCES)))

ABDUCER_ICE_VERSION:=$(shell ./svn-file-version $(SLICE_PATH)/Abducer.ice)

CXXFLAGS+=-I. -I$(ICE_HOME)/include
CXXFLAGS+=-DABDUCER_ICE_VERSION=\"$(ABDUCER_ICE_VERSION)\"

$(BIN): $(HEADERS) $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

Abducer.h Abducer.cc: $(SLICE_PATH)/Abducer.ice
	slice2cpp -I$(SLICE_PATH) --source-ext cc $(SLICE_PATH)/Abducer.ice

%.o: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

.PHONY: clean
clean:
	rm -f $(OBJS) $(BIN) Abducer.cc Abducer.h
