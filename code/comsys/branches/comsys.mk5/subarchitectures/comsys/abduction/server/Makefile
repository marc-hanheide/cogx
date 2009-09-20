# $Id$

# Binary files to be built.
#
BIN=abducer-server

# Mercury source files.
#
MFILES=abducer-server.m iceserv_lib.m

# Mercury compiler flags
#
MCFLAGS?=--search-lib-files-dir ../lib \
		--init-file ../lib/abducer.init \
		--link-object ../lib/libabducer.a

#------------------------------------------------------------------------------#

SLICE_PATH?=..

ICE_HOME=$(HOME)/Developer/Ice-3.3.1

MERCURY_HOME=$(HOME)/Developer/mercury-rotd-2008-11-30

CXXFLAGS=-Wall
LDFLAGS=-L$(ICE_HOME)/lib -lIce -lIceUtil

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

.PHONY: compile
compile: $(BIN)

.PRECIOUS: %.o
%.o: %.cc Abducer.h aserv.h iceserv_lib.mh
	$(CXX) $(CXXFLAGS) -I. -I$(ICE_HOME)/include -I$(MERCURY_HOME)/lib/mercury/inc -I$(MERCURY_HOME)/lib/mercury/conf -c $*.cc

Abducer.h Abducer.cc: $(SLICE_PATH)/Abducer.ice
	slice2cpp -I . --source-ext cc ../Abducer.ice

abducer-server: $(MFILES) aserv.o Abducer.o
	mmc --make \
		--link-executable-command=$(CXX) \
		--link-shared-lib-command=$(CXX) \
		--link-object aserv.o \
		--link-object Abducer.o \
		--grade $(GRADE) \
		$(MCFLAGS) \
		$(LDFLAGS) \
		$@

.PRECIOUS: %.mh
%.mh: %.m
	mmc --make \
		--grade $(GRADE) \
		$(MCFLAGS) \
		$*.o

.PHONY: clean
clean:
	rm -rf $(BIN) Mercury \
		$(addsuffix .err, $(basename $(MFILES))) \
		$(addsuffix .mh, $(basename $(MFILES))) \
		abducer-server aserv.o \
		Abducer.o Abducer.cc Abducer.h
