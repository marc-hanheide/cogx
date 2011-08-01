HEADERS = heuristic.h \
	  cyclic_cg_heuristic.h no_heuristic.h domain_transition_graph.h globals.h \
          operator.h state.h successor_generator.h causal_graph.h \
	  best_first_search.h axioms.h cache.h search_engine.h closed_list.h \
	  scheduler.h partial_order_lifter.h monitoring.h

SOURCES = planner.cc $(HEADERS:%.h=%.cc)
OBJECTS = $(SOURCES:%.cc=%.o)
PROFILE_OBJECTS = $(SOURCES:%.cc=%.profile.o)

CC = g++
DEPEND = g++ -MM
CCOPT = -Wall -W -Wno-sign-compare -ansi -pedantic # -Werror
LINKOPT = 

## debug, don't optimize
#CCOPT += -g3
#LINKOPT += -g3

## debug, optimize
#CCOPT += -O3

## no debug, optimize a lot
CCOPT += -O3 -DNDEBUG

PROFOPT = -pg
# PROFOPT = -pg -O3
# PROFLINKOPT = -lc_p
# PROFARGS = pP < ~/planning/results/preprocess/grid/prob04.pre
 PROFARGS = yY t 70 plan.soln < output
# PROFARGS = n < ../preprocess/output

vpath %.o obj

search: $(OBJECTS)
	$(CC) $(LINKOPT) $(OBJECTS:%=obj/%) -o search

profile-search: $(PROFILE_OBJECTS)
	$(CC) $(LINKOPT) $(PROFLINKOPT) $(PROFOPT) $(PROFILE_OBJECTS:%=obj/%) \
          -o profile-search


PROFILE: profile-search
	./profile-search $(PROFARGS)
	gprof profile-search > PROFILE

$(OBJECTS): %.o: %.cc
	$(CC) $(CCOPT) -c $< -o obj/$@

$(PROFILE_OBJECTS): %.profile.o: %.cc
	$(CC) $(CCOPT) $(PROFOPT) -c $< -o obj/$@

clean:
	rm -f $(OBJECTS:%=obj/%) $(PROFILE_OBJECTS:%=obj/%) *~ *.pyc
	rm -f Makefile.depend profile-search gmon.out PROFILE core
	rm -f sas_plan

veryclean: clean
	rm -f search

Makefile.depend: $(SOURCES) $(HEADERS)
	$(DEPEND) $(SOURCES) > Makefile.temp
	sed -e "s/\(.*\)\.o:\(.*\)/\1.profile.o:\2/" Makefile.temp >> Makefile.temp2
	cat Makefile.temp Makefile.temp2 > Makefile.depend
	rm -f Makefile.temp Makefile.temp2

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),veryclean)
-include Makefile.depend
endif
endif

