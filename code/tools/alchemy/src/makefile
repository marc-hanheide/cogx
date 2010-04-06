SHELL=/bin/bash
INLINE_LIMIT=-finline-limit=100000
#INLINE_WARN=-Winline
STATIC=-dynamic

######### Profile
#CLFLAGS=-pg -g $(STATIC) -DNDEBUG $(INLINE_WARN) $(INLINE_LIMIT) 

######### Profile & optimize
#CLFLAGS=-pg -g -O3 $(STATIC) -DNDEBUG $(INLINE_WARN) $(INLINE_LIMIT) 

######## Optimize
#CLFLAGS=-O3 $(STATIC) $(INLINE_WARN) $(INLINE_LIMIT)

######## Optimize and turn off asserts
CLFLAGS=-O3 -DNDEBUG $(STATIC) $(INLINE_WARN) $(INLINE_LIMIT)

########## Debug
#CLFLAGS=-g

BASEDIR=..

GPP=g++
FLEX=flex
BISON=bison

BIN=$(BASEDIR)/bin
OBJDIR=$(BASEDIR)/bin/obj
MF=makefile

SRCDIR=$(BASEDIR)/src
UTIL=$(SRCDIR)/util
PARSER=$(SRCDIR)/parser
LOGIC=$(SRCDIR)/logic
INFER=$(SRCDIR)/infer
LEARNWTS=$(SRCDIR)/learnwts
LEARNSTRUCT=$(SRCDIR)/learnstruct

REPLACEFOLCPP=$(PARSER)/replacefolcpp.pl

LIBCINC= -I. -I$(UTIL) -I$(LOGIC) -I$(PARSER) -I$(LEARNWTS) -I$(LEARNSTRUCT) \
	-I$(INFER)
LIBLINC= -L. -L$(UTIL) -L$(LOGIC) -L$(PARSER) -L$(LEARNWTS) -L$(LEARNSTRUCT) \
	-L$(INFER)
LIBLINK= -ldl

CFLAGS=$(CLFLAGS) -Wno-deprecated -Wall $(LIBCINC)
LFLAGS=$(CLFLAGS) -Wno-deprecated -Wall $(LIBLINC)


#############################################################################
#add non-executables .cpp files to OBJS 
#if .cpp file reside in a dir other than the current one, add .o rule below
OBJS=$(OBJDIR)/arguments.o $(OBJDIR)/powerset.o \
	$(OBJDIR)/fol.o $(OBJDIR)/listobj.o \
	$(OBJDIR)/predicatetemplate.o $(OBJDIR)/functiontemplate.o \
	$(OBJDIR)/function.o $(OBJDIR)/predicate.o $(OBJDIR)/term.o \
	$(OBJDIR)/domain.o $(OBJDIR)/clause.o $(OBJDIR)/clausesampler.o \
	$(OBJDIR)/Polynomial.o $(OBJDIR)/random.o \
	$(OBJDIR)/logichelper.o \
	$(OBJDIR)/groundclause.o $(OBJDIR)/superpred.o \
	$(OBJDIR)/superclause.o \
	$(OBJDIR)/node.o $(OBJDIR)/factor.o

#add .cpp files (including executables') to CPPFILES.
CPPFILES=$(UTIL)/arguments.cpp $(UTIL)/powerset.cpp $(UTIL)/random.cpp\
	$(PARSER)/fol.cpp $(PARSER)/listobj.cpp \
	$(LOGIC)/predicatetemplate.cpp $(LOGIC)/functiontemplate.cpp \
	$(LOGIC)/function.cpp $(LOGIC)/predicate.cpp $(LOGIC)/term.cpp \
	$(LOGIC)/domain.cpp  $(LOGIC)/clause.cpp $(LOGIC)/clausesampler.cpp\
	$(INFER)/groundclause.cpp \
	$(LEARNWTS)/learnwts.cpp $(INFER)/infer.cpp \
	$(LEARNSTRUCT)/learnstruct.cpp \
	$(LOGIC)/Polynomial.cpp\
	$(LOGIC)/logichelper.cpp\
	$(INFER)/superpred.cpp \
	$(INFER)/superclause.cpp \
	$(INFER)/node.cpp $(INFER)/factor.cpp

##############################################################################
#EXECUTABLES: add executables rules here

all: learnwts infer learnstruct

learnwts: $(OBJDIR)/learnwts.oo
	cp $(OBJDIR)/learnwts.oo $(BIN)/learnwts

infer: $(OBJDIR)/infer.oo
	cp $(OBJDIR)/infer.oo $(BIN)/infer

learnstruct: $(OBJDIR)/learnstruct.oo
	cp $(OBJDIR)/learnstruct.oo $(BIN)/learnstruct

$(OBJDIR)/learnwts.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/learnwts.o -c $(LEARNWTS)/learnwts.cpp

$(OBJDIR)/infer.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/infer.o -c $(INFER)/infer.cpp

$(OBJDIR)/learnstruct.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/learnstruct.o -c $(LEARNSTRUCT)/learnstruct.cpp

%.oo: %.o $(OBJS) $(LIBS)
	echo $<
	$(GPP) $(LFLAGS) -o $@ $(OBJS) $< $(LIBLINK)


##############################################################################
#add add non-executables .o rules here

$(OBJDIR)/fol.o: $(PARSER)/fol.y $(PARSER)/follex.y
	$(FLEX) -o$(PARSER)/follex.cpp $(PARSER)/follex.y 
	$(BISON) $(PARSER)/fol.y;
	mv -f fol.tab.c  $(PARSER)/fol.cpp
	$(REPLACEFOLCPP) $(PARSER)/fol.cpp
	$(GPP) $(CFLAGS) -o $(OBJDIR)/fol.o -c $(PARSER)/fol.cpp


#.o rule
#$(OBJDIR)/file.o:
#	$(GPP) $(CFLAGS) -o $(OBJDIR)/file.o -c dir/file.cpp


$(OBJDIR)/arguments.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/arguments.o -c $(UTIL)/arguments.cpp

$(OBJDIR)/powerset.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/powerset.o -c $(UTIL)/powerset.cpp

$(OBJDIR)/random.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/random.o -c $(UTIL)/random.cpp

$(OBJDIR)/listobj.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/listobj.o -c $(PARSER)/listobj.cpp 

$(OBJDIR)/predicatetemplate.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/predicatetemplate.o -c $(LOGIC)/predicatetemplate.cpp 

$(OBJDIR)/functiontemplate.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/functiontemplate.o -c $(LOGIC)/functiontemplate.cpp 

$(OBJDIR)/function.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/function.o -c $(LOGIC)/function.cpp 

$(OBJDIR)/predicate.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/predicate.o -c $(LOGIC)/predicate.cpp 

$(OBJDIR)/term.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/term.o -c $(LOGIC)/term.cpp

$(OBJDIR)/domain.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/domain.o -c $(LOGIC)/domain.cpp  

$(OBJDIR)/clause.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/clause.o -c $(LOGIC)/clause.cpp  

$(OBJDIR)/clausesampler.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/clausesampler.o -c $(LOGIC)/clausesampler.cpp  

$(OBJDIR)/groundclause.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/groundclause.o -c $(INFER)/groundclause.cpp

$(OBJDIR)/Polynomial.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/Polynomial.o -c $(LOGIC)/Polynomial.cpp
	
$(OBJDIR)/logichelper.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/logichelper.o -c $(LOGIC)/logichelper.cpp

$(OBJDIR)/superpred.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/superpred.o -c $(INFER)/superpred.cpp

$(OBJDIR)/superclause.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/superclause.o -c $(INFER)/superclause.cpp

$(OBJDIR)/node.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/node.o -c $(INFER)/node.cpp

$(OBJDIR)/factor.o:
	$(GPP) $(CFLAGS) -o $(OBJDIR)/factor.o -c $(INFER)/factor.cpp

###############################################################################


clean:
	@rm -f $(OBJDIR)/*.o $(OBJDIR)/*.oo

depend:
	@echo 'updating the dependencies for:'
	@echo '    ' $(CPPFILES)
	@( \
	< $(MF) sed -n '1,/^###.*SUDDEN DEATH/p'; \
	    echo '#' ; \
	    echo '# dependencies generated on: ' `date` ; \
	    echo '#' ; \
	    for i in $(CPPFILES); do \
	        echo -n "$(OBJDIR)/" ; \
	        $(GPP) -MM $(CFLAGS) $$i ; \
	        echo; \
	    done \
	) > $(MF).new
	@mv $(MF) $(MF).last
	@mv $(MF).new $(MF)

############ Everything below here is subject to SUDDEN DEATH #####
#
# dependencies generated on:  Di 6 Apr 2010 14:11:49 CEST
#
../bin/obj/arguments.o: ../src/util/arguments.cpp ../src/util/arguments.h

../bin/obj/powerset.o: ../src/util/powerset.cpp ../src/util/powerset.h \
  ../src/util/array.h ../src/util/random.h

../bin/obj/random.o: ../src/util/random.cpp ../src/util/random.h

../bin/obj/fol.o: ../src/parser/fol.cpp ../src/parser/fol.h ../src/logic/mln.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/random.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/parser/follex.cpp ../src/parser/folhelper.h \
  ../src/parser/strfifolist.h ../src/logic/clausefactory.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/parser/listobj.h \
  ../src/util/timer.h ../src/parser/internals.h

../bin/obj/listobj.o: ../src/parser/listobj.cpp ../src/parser/listobj.h \
  ../src/util/array.h ../src/util/random.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h \
  ../src/util/arraysaccessor.h ../src/util/array.h

../bin/obj/predicatetemplate.o: ../src/logic/predicatetemplate.cpp \
  ../src/logic/predicatetemplate.h ../src/util/array.h \
  ../src/util/random.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/equalstr.h \
  ../src/util/constdualmap.h ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h

../bin/obj/functiontemplate.o: ../src/logic/functiontemplate.cpp \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/util/array.h ../src/util/random.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h

../bin/obj/function.o: ../src/logic/function.cpp ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/util/array.h ../src/util/random.h ../src/util/hash.h \
  ../src/util/array.h ../src/logic/term.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/function.h

../bin/obj/predicate.o: ../src/logic/predicate.cpp ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/util/array.h \
  ../src/util/random.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/equalstr.h \
  ../src/util/constdualmap.h ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicate.h ../src/logic/function.h \
  ../src/util/hashstring.h ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h

../bin/obj/term.o: ../src/logic/term.cpp ../src/logic/term.h ../src/util/array.h \
  ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/util/hash.h ../src/util/array.h ../src/logic/term.h \
  ../src/logic/domain.h ../src/util/equalstr.h ../src/util/dualmap.h \
  ../src/util/array.h ../src/util/equalstr.h ../src/util/constdualmap.h \
  ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/function.h

../bin/obj/domain.o: ../src/logic/domain.cpp ../src/logic/clause.h \
  ../src/logic/domain.h ../src/util/equalstr.h ../src/util/dualmap.h \
  ../src/util/array.h ../src/util/random.h ../src/util/equalstr.h \
  ../src/util/constdualmap.h ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/domain.h \
  ../src/logic/database.h ../src/logic/mln.h ../src/logic/clause.h \
  ../src/logic/mlnhelper.h ../src/logic/truefalsegroundingsstore.h

../bin/obj/clause.o: ../src/logic/clause.cpp ../src/logic/clause.h \
  ../src/logic/domain.h ../src/util/equalstr.h ../src/util/dualmap.h \
  ../src/util/array.h ../src/util/random.h ../src/util/equalstr.h \
  ../src/util/constdualmap.h ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/infer/mrf.h ../src/util/timer.h \
  ../src/logic/mln.h ../src/logic/clause.h ../src/logic/mlnhelper.h \
  ../src/infer/groundpredicate.h ../src/infer/variable.h \
  ../src/util/util.h ../src/infer/superclause.h ../src/infer/mrf.h \
  ../src/infer/variable.h

../bin/obj/clausesampler.o: ../src/logic/clausesampler.cpp \
  ../src/logic/clausesampler.h ../src/logic/database.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/random.h ../src/logic/groundpreds.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h \
  ../src/logic/domain.h ../src/util/equalstr.h ../src/util/dualmap.h \
  ../src/util/array.h ../src/util/equalstr.h ../src/util/constdualmap.h \
  ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicate.h ../src/logic/function.h \
  ../src/util/arraysaccessor.h ../src/util/array.h \
  ../src/infer/groundpredicate.h ../src/infer/groundclause.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/predicate.h ../src/logic/clausesampler.h \
  ../src/logic/clausehelper.h

../bin/obj/groundclause.o: ../src/infer/groundclause.cpp ../src/infer/groundclause.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/util/array.h \
  ../src/util/random.h ../src/util/hash.h ../src/util/array.h \
  ../src/infer/groundpredicate.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/infer/groundclause.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mln.h ../src/logic/clause.h \
  ../src/logic/mlnhelper.h

../bin/obj/learnwts.o: ../src/learnwts/learnwts.cpp ../src/util/arguments.h \
  ../src/infer/inferenceargs.h ../src/learnwts/lbfgsb.h \
  ../src/util/timer.h ../src/learnwts/pseudologlikelihood.h \
  ../src/util/array.h ../src/util/random.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/logic/clause.h ../src/logic/domain.h \
  ../src/util/hashstring.h ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mln.h ../src/logic/clause.h \
  ../src/logic/mlnhelper.h ../src/learnwts/indextranslator.h \
  ../src/logic/Polynomial.h ../src/learnwts/discriminativelearner.h \
  ../src/infer/infer.h ../src/util/util.h ../src/parser/fol.h \
  ../src/infer/mrf.h ../src/infer/groundpredicate.h \
  ../src/learnwts/learnwts.h ../src/learnwts/indextranslator.h \
  ../src/infer/inferenceargs.h ../src/infer/maxwalksat.h \
  ../src/infer/sat.h ../src/infer/inference.h \
  ../src/logic/variablestate.h ../src/logic/hvariablestate.h \
  ../src/logic/Polynomial.h ../src/learnwts/lbfgsp.h \
  ../src/logic/logsigmoid.h ../src/logic/logichelper.h \
  ../src/infer/maxwalksatparams.h ../src/infer/samplesatparams.h \
  ../src/infer/mcsat.h ../src/infer/mcmc.h ../src/infer/inference.h \
  ../src/infer/mcmcparams.h ../src/infer/mcsatparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/unitpropagation.h ../src/infer/inference.h \
  ../src/infer/maxwalksat.h ../src/infer/gibbssampler.h \
  ../src/infer/mcmc.h ../src/infer/gibbsparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/maxwalksat.h ../src/infer/convergencetest.h \
  ../src/util/meanvariance.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/simulatedtempering.h ../src/infer/mcmc.h \
  ../src/infer/simulatedtemperingparams.h ../src/infer/mcmcparams.h \
  ../src/infer/maxwalksatparams.h ../src/infer/maxwalksat.h \
  ../src/infer/convergencetest.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/bp.h ../src/infer/inference.h ../src/infer/bpparams.h \
  ../src/infer/twowaymessage.h ../src/infer/superclause.h \
  ../src/infer/mrf.h ../src/infer/variable.h ../src/infer/auxfactor.h \
  ../src/infer/factor.h ../src/infer/mrf.h ../src/infer/link.h \
  ../src/infer/mrf.h ../src/infer/node.h ../src/infer/mrf.h \
  ../src/infer/link.h ../src/infer/superpred.h ../src/infer/superclause.h \
  ../src/infer/twowaymessage.h ../src/infer/factor.h \
  ../src/infer/superclause.h ../src/infer/node.h \
  ../src/infer/factorgraph.h ../src/infer/twowaymessage.h \
  ../src/infer/superclause.h ../src/infer/auxfactor.h ../src/infer/node.h \
  ../src/infer/hmcsat.h ../src/infer/mcmc.h ../src/infer/mcsatparams.h \
  ../src/infer/hmaxwalksat.h ../src/infer/sat.h \
  ../src/infer/maxwalksatparams.h ../src/learnwts/indextranslator.h \
  ../src/learnwts/learnwts.h

../bin/obj/infer.o: ../src/infer/infer.cpp ../src/parser/fol.h ../src/logic/mln.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/random.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/util/arguments.h ../src/util/util.h ../src/infer/infer.h \
  ../src/infer/mrf.h ../src/util/timer.h ../src/infer/groundpredicate.h \
  ../src/learnwts/learnwts.h ../src/learnwts/indextranslator.h \
  ../src/infer/inferenceargs.h ../src/infer/maxwalksat.h \
  ../src/infer/sat.h ../src/infer/inference.h \
  ../src/logic/variablestate.h ../src/logic/hvariablestate.h \
  ../src/logic/Polynomial.h ../src/learnwts/lbfgsp.h \
  ../src/logic/logsigmoid.h ../src/logic/logichelper.h \
  ../src/infer/maxwalksatparams.h ../src/infer/samplesatparams.h \
  ../src/infer/mcsat.h ../src/infer/mcmc.h ../src/infer/inference.h \
  ../src/infer/mcmcparams.h ../src/infer/mcsatparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/unitpropagation.h ../src/infer/inference.h \
  ../src/infer/maxwalksat.h ../src/infer/gibbssampler.h \
  ../src/infer/mcmc.h ../src/infer/gibbsparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/maxwalksat.h ../src/infer/convergencetest.h \
  ../src/util/meanvariance.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/simulatedtempering.h ../src/infer/mcmc.h \
  ../src/infer/simulatedtemperingparams.h ../src/infer/mcmcparams.h \
  ../src/infer/maxwalksatparams.h ../src/infer/maxwalksat.h \
  ../src/infer/convergencetest.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/bp.h ../src/infer/inference.h ../src/infer/bpparams.h \
  ../src/infer/twowaymessage.h ../src/infer/superclause.h \
  ../src/infer/mrf.h ../src/infer/variable.h ../src/infer/auxfactor.h \
  ../src/infer/factor.h ../src/infer/mrf.h ../src/infer/link.h \
  ../src/infer/mrf.h ../src/infer/node.h ../src/infer/mrf.h \
  ../src/infer/link.h ../src/infer/superpred.h ../src/infer/superclause.h \
  ../src/infer/twowaymessage.h ../src/infer/factor.h \
  ../src/infer/superclause.h ../src/infer/node.h \
  ../src/infer/factorgraph.h ../src/infer/twowaymessage.h \
  ../src/infer/superclause.h ../src/infer/auxfactor.h ../src/infer/node.h \
  ../src/infer/hmcsat.h ../src/infer/mcmc.h ../src/infer/mcsatparams.h \
  ../src/infer/hmaxwalksat.h ../src/infer/sat.h \
  ../src/infer/maxwalksatparams.h

../bin/obj/learnstruct.o: ../src/learnstruct/learnstruct.cpp ../src/util/arguments.h \
  ../src/parser/fol.h ../src/logic/mln.h ../src/logic/clause.h \
  ../src/logic/domain.h ../src/util/equalstr.h ../src/util/dualmap.h \
  ../src/util/array.h ../src/util/random.h ../src/util/equalstr.h \
  ../src/util/constdualmap.h ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/learnwts/learnwts.h ../src/util/util.h ../src/util/timer.h \
  ../src/learnwts/indextranslator.h ../src/learnstruct/structlearn.h \
  ../src/logic/clausefactory.h ../src/logic/clause.h \
  ../src/logic/domain.h ../src/learnwts/lbfgsb.h \
  ../src/learnwts/pseudologlikelihood.h ../src/learnwts/indextranslator.h \
  ../src/logic/Polynomial.h ../src/learnwts/discriminativelearner.h \
  ../src/infer/infer.h ../src/infer/mrf.h ../src/infer/groundpredicate.h \
  ../src/infer/inferenceargs.h ../src/infer/maxwalksat.h \
  ../src/infer/sat.h ../src/infer/inference.h \
  ../src/logic/variablestate.h ../src/logic/hvariablestate.h \
  ../src/logic/Polynomial.h ../src/learnwts/lbfgsp.h \
  ../src/logic/logsigmoid.h ../src/logic/logichelper.h \
  ../src/infer/maxwalksatparams.h ../src/infer/samplesatparams.h \
  ../src/infer/mcsat.h ../src/infer/mcmc.h ../src/infer/inference.h \
  ../src/infer/mcmcparams.h ../src/infer/mcsatparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/unitpropagation.h ../src/infer/inference.h \
  ../src/infer/maxwalksat.h ../src/infer/gibbssampler.h \
  ../src/infer/mcmc.h ../src/infer/gibbsparams.h \
  ../src/infer/mcmcparams.h ../src/infer/maxwalksatparams.h \
  ../src/infer/maxwalksat.h ../src/infer/convergencetest.h \
  ../src/util/meanvariance.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/simulatedtempering.h ../src/infer/mcmc.h \
  ../src/infer/simulatedtemperingparams.h ../src/infer/mcmcparams.h \
  ../src/infer/maxwalksatparams.h ../src/infer/maxwalksat.h \
  ../src/infer/convergencetest.h ../src/infer/gelmanconvergencetest.h \
  ../src/infer/bp.h ../src/infer/inference.h ../src/infer/bpparams.h \
  ../src/infer/twowaymessage.h ../src/infer/superclause.h \
  ../src/infer/mrf.h ../src/infer/variable.h ../src/infer/auxfactor.h \
  ../src/infer/factor.h ../src/infer/mrf.h ../src/infer/link.h \
  ../src/infer/mrf.h ../src/infer/node.h ../src/infer/mrf.h \
  ../src/infer/link.h ../src/infer/superpred.h ../src/infer/superclause.h \
  ../src/infer/twowaymessage.h ../src/infer/factor.h \
  ../src/infer/superclause.h ../src/infer/node.h \
  ../src/infer/factorgraph.h ../src/infer/twowaymessage.h \
  ../src/infer/superclause.h ../src/infer/auxfactor.h ../src/infer/node.h \
  ../src/infer/hmcsat.h ../src/infer/mcmc.h ../src/infer/mcsatparams.h \
  ../src/infer/hmaxwalksat.h ../src/infer/sat.h \
  ../src/infer/maxwalksatparams.h ../src/learnwts/indextranslator.h

../bin/obj/Polynomial.o: ../src/logic/Polynomial.cpp ../src/logic/Polynomial.h \
  ../src/util/array.h ../src/util/random.h ../src/util/complex.h \
  ../src/util/random.h ../src/util/array.h

../bin/obj/logichelper.o: ../src/logic/logichelper.cpp ../src/logic/logichelper.h \
  ../src/util/array.h ../src/util/random.h

../bin/obj/superpred.o: ../src/infer/superpred.cpp ../src/util/util.h \
  ../src/infer/mrf.h ../src/util/timer.h ../src/logic/mln.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/random.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/infer/groundpredicate.h ../src/infer/superclause.h \
  ../src/infer/mrf.h ../src/infer/variable.h ../src/infer/superpred.h \
  ../src/infer/superclause.h

../bin/obj/superclause.o: ../src/infer/superclause.cpp ../src/util/util.h \
  ../src/infer/mrf.h ../src/util/timer.h ../src/logic/mln.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/random.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/infer/groundpredicate.h ../src/infer/superclause.h \
  ../src/infer/mrf.h ../src/infer/variable.h ../src/infer/superpred.h \
  ../src/infer/superclause.h

../bin/obj/node.o: ../src/infer/node.cpp ../src/infer/node.h ../src/util/util.h \
  ../src/infer/mrf.h ../src/util/timer.h ../src/logic/mln.h \
  ../src/logic/clause.h ../src/logic/domain.h ../src/util/equalstr.h \
  ../src/util/dualmap.h ../src/util/array.h ../src/util/random.h \
  ../src/util/equalstr.h ../src/util/constdualmap.h ../src/util/array.h \
  ../src/util/strint.h ../src/logic/predicatetemplate.h \
  ../src/logic/functiontemplate.h ../src/logic/predicatetemplate.h \
  ../src/logic/predicate.h ../src/logic/predicatetemplate.h \
  ../src/logic/term.h ../src/util/hash.h ../src/util/array.h \
  ../src/util/hasharray.h ../src/util/random.h ../src/logic/function.h \
  ../src/logic/functiontemplate.h ../src/logic/term.h \
  ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/infer/groundpredicate.h ../src/infer/link.h ../src/infer/mrf.h \
  ../src/infer/superpred.h ../src/infer/superclause.h ../src/infer/mrf.h \
  ../src/infer/variable.h ../src/infer/twowaymessage.h \
  ../src/infer/factor.h ../src/infer/mrf.h ../src/infer/link.h \
  ../src/infer/node.h ../src/infer/superclause.h

../bin/obj/factor.o: ../src/infer/factor.cpp ../src/infer/factor.h \
  ../src/util/util.h ../src/infer/mrf.h ../src/util/timer.h \
  ../src/logic/mln.h ../src/logic/clause.h ../src/logic/domain.h \
  ../src/util/equalstr.h ../src/util/dualmap.h ../src/util/array.h \
  ../src/util/random.h ../src/util/equalstr.h ../src/util/constdualmap.h \
  ../src/util/array.h ../src/util/strint.h \
  ../src/logic/predicatetemplate.h ../src/logic/functiontemplate.h \
  ../src/logic/predicatetemplate.h ../src/logic/predicate.h \
  ../src/logic/predicatetemplate.h ../src/logic/term.h ../src/util/hash.h \
  ../src/util/array.h ../src/util/hasharray.h ../src/util/random.h \
  ../src/logic/function.h ../src/logic/functiontemplate.h \
  ../src/logic/term.h ../src/logic/function.h ../src/util/hashstring.h \
  ../src/util/hasharray.h ../src/util/hashlist.h \
  ../src/util/arraysaccessor.h ../src/util/array.h ../src/util/powerset.h \
  ../src/util/array.h ../src/util/multdarray.h ../src/util/array.h \
  ../src/util/hashint.h ../src/util/hasharray.h ../src/logic/database.h \
  ../src/logic/groundpreds.h ../src/logic/predicate.h \
  ../src/logic/domain.h ../src/infer/groundpredicate.h \
  ../src/infer/groundclause.h ../src/logic/predicate.h \
  ../src/logic/clausesampler.h ../src/logic/database.h \
  ../src/logic/truefalsegroundingsstore.h ../src/logic/domain.h \
  ../src/logic/clausehelper.h ../src/logic/mlnhelper.h \
  ../src/infer/groundpredicate.h ../src/infer/link.h ../src/infer/mrf.h \
  ../src/infer/node.h ../src/infer/mrf.h ../src/infer/link.h \
  ../src/infer/superpred.h ../src/infer/superclause.h ../src/infer/mrf.h \
  ../src/infer/variable.h ../src/infer/twowaymessage.h \
  ../src/infer/factor.h ../src/infer/superclause.h

