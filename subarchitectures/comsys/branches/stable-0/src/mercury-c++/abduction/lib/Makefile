# $Id$

# Binary files to be built.
#
MLIB=libabducer

# Mercury source files.
#
MFILES=abduction.m context.m formula.m costs.m modality.m stringable.m ctx_modality.m formula_io.m lf.m ctx_specific.m ctx_io.m formula_ops.m ctx_modality.m rcc.m ctx_loadable.m belief_model.m model.m lf_io.m stf.m ontology.m model_io.m ling.m enumerable.m ctx_ontology.m utils.m ctx_specific_io.m ctx_loadable_io.m abducer.m

# Mercury compiler flags
#
MCFLAGS?=

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

ifeq ($(origin DEBUG), undefined)
else
MCFLAGS+=--trace-flag=debug
MCFLAGS+=--cflag -g
endif

# on BSD, without deactivating suffix rules, any "non-suffixed"
# target would be matched by some sort of default rule (using
# $(CC)), not by our rule.
.SUFFIXES:

.PHONY: compile
compile: $(MLIB)

$(MLIB): $(MFILES)
	mmc --make \
		--link-shared-lib-command=$(CXX) \
		--lib-linkage static \
		--grade $(GRADE) \
		--libgrade $(GRADE) \
		$(MCFLAGS) \
		$(LDFLAGS) \
		$@

.PHONY: clean
clean:
	rm -rf $(BIN) Mercury \
		$(addsuffix .err, $(basename $(MFILES))) \
		$(addsuffix .mh, $(basename $(MFILES))) \
		libabducer.a abducer.init
