# $Id$

# Binary files to be built.
#
BIN=test_abduction test_formulae test_lf test_modality

# Mercury source files.
#
MFILES=test_abduction.m abduction.m context.m formula.m costs.m test_formulae.m modality.m stringable.m ctx_modality.m formula_io.m lf.m test_lf.m test_modality.m ctx_impl.m

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
