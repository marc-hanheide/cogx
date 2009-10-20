#------------------------------------------------------------------------------#

# ICE

SLICE_PATH?=../../../slice

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# C++

BIN=abducer-server-mk2

CXX_HEADERS=Tokens.h TermTokeniser.h TermParsing.h Terms.h TtyUtils.h ForwardedAbducerServer.h
CXX_HEADERS+=Abducer.h

CXX_SOURCES=Tokens.cc TermTokeniser.cc ServerMain.cc TermParsing.cc Terms.cc TtyUtils.cc
CXX_SOURCES+=Abducer.cc

CXXFLAGS=-Wall
CXXFLAGS+=-I. -I$(ICE_HOME)/include
CXXFLAGS+=-DABDUCER_ICE_VERSION=\"$(ABDUCER_ICE_VERSION)\"

LDFLAGS=-L$(ICE_HOME)/lib -lIce -lIceUtil

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

# Mercury

MBIN=abducer-cli
M_SOURCES=abducer-cli.m

LIBABDUCER=../lib/libabducer.a

MCFLAGS?=--search-lib-files-dir ../lib \
		--init-file ../lib/abducer.init \
		--link-object $(LIBABDUCER)

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.

ifeq ($(shell uname -s), Darwin)
# on Darwin, Mercury doesn't support asm_fast (although it should for
# Intel-based systems?)
GRADE?=hlc.gc
else
# try asm_fast.gc otherwise
GRADE?=asm_fast.gc
endif

# on BSD, without deactivating suffix rules, any "non-suffixed"
# target would be matched by some sort of default rule (using
# $(CC)), not by our rule.
.SUFFIXES:

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

ABDUCER_ICE_VERSION:=$(shell ./svn-file-version $(SLICE_PATH)/Abducer.ice)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: compile
compile: $(BIN) $(MBIN)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

Abducer.h Abducer.cc: $(SLICE_PATH)/Abducer.ice
	slice2cpp -I$(SLICE_PATH) --source-ext cc $(SLICE_PATH)/Abducer.ice

%.o: %.cc $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

OBJS=$(addsuffix .o, $(basename $(CXX_SOURCES)))

$(BIN): $(CXX_HEADERS) $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

$(MBIN): $(M_SOURCES) $(LIBABDUCER)
	mmc --make \
		--grade $(GRADE) \
		$(MCFLAGS) \
		$@

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	rm -rf $(OBJS) Abducer.cc Abducer.h \
		Mercury \
		$(addsuffix .err, $(basename $(M_SOURCES))) \
		$(addsuffix .mh, $(basename $(M_SOURCES))) \
		$(BIN) $(MBIN)
