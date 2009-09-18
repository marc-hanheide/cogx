# $Id$

# Binary files to be built.
#
BIN=test_abduction test_formulae test_lf test_modality test_rcc iceserv

# Mercury source files.
#
MFILES=test_abduction.m abduction.m context.m formula.m costs.m test_formulae.m modality.m stringable.m ctx_modality.m formula_io.m lf.m test_lf.m test_modality.m ctx_impl.m ctx_io.m formula_ops.m ctx_modality.m rcc.m test_rcc.m iceserv.m iceserv_lib.m

# Mercury compiler flags
#
MCFLAGS?=

#------------------------------------------------------------------------------#

ICE_HOME=$(HOME)/Developer/Ice-3.3.1

MERCURY_HOME=$(HOME)/Developer/mercury-rotd-2008-11-30

CXXFLAGS=-Wall
LDFLAGS=-L$(ICE_HOME)/lib -lIce -lIceUtil

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.

# use a default grade if unset
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

aserv: aserv.o Abducer.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o aserv Abducer.o aserv.o

.PRECIOUS: %.o
%.o: %.cc Abducer.h aserv.h
	$(CXX) $(CXXFLAGS) -I. -I$(ICE_HOME)/include -I$(MERCURY_HOME)/lib/mercury/inc -I$(MERCURY_HOME)/lib/mercury/conf -c $*.cc

Abducer.h Abducer.cc: Abducer.ice
	slice2cpp --source-ext cc Abducer.ice

$(BIN): $(MFILES) aserv.o
	mmc --make \
		--link-executable-command=$(CXX) \
		--link-shared-lib-command=$(CXX) \
		--link-object aserv.o \
		--link-object Abducer.o \
		--grade $(GRADE) \
		$(MCFLAGS) \
		$(LDFLAGS) \
		$@

.PHONY: clean
clean:
	rm -rf $(BIN) Mercury \
		$(addsuffix .err, $(basename $(MFILES))) \
		$(addsuffix .mh, $(basename $(MFILES))) \
		aserv aserv.o Abducer.o Abducer.cc Abducer.h
