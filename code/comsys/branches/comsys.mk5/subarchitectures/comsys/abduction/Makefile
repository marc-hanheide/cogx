# $Id$

# Binary files to be built.
#
BIN=test_abduction test_formulae

# Mercury source files.
#
MFILES=test_abduction.m abduction.m kb.m context.m formula.m costs.m test_formulae.m

# Mercury compiler flags
#
MCFLAGS?=

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

$(BIN): $(MFILES)
	mmc --make \
		--grade $(GRADE) \
		$(MCFLAGS) \
		$@

.PHONY: clean
clean:
	rm -rf $(BIN) Mercury \
		$(addsuffix .err, $(basename $(MFILES))) \
		$(addsuffix .mh, $(basename $(MFILES)))
