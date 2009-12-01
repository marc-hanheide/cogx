TARGET=ayPlan

FLEX_LIB=-L/usr/ -lfl -lcurses

RANLIB	= ranlib

CXX =  g++ -DOLD_FLEX # ccache g++ -DOLD_FLEX -DNDEBUG -O3 -fno-strict-aliasing -fomit-frame-pointer -funroll-all-loops -fexpensive-optimizations   -mno-align-stringops  -minline-all-stringops -pipe  -mfpmath=sse -march=nocona -m64 -msse3 -falign-functions=64 -fforce-addr -ffast-math -mpush-args  -maccumulate-outgoing-args -momit-leaf-frame-pointer

GENERATED=pddl+

MODULES= Functions Curses Action Arguments Domain global PredicatesAndPropositions Problem Planner GroundAction State GroundActionWithCost ActionCostState CostPlanner StateEvaluation UnexpandedStack

EXTRA_MODULES_EXECUTABLE= main

EXTRA_OBJECTS_EXECUTABLE=$(EXTRA_MODULES_EXECUTABLE:=.o)

OBJECTS=$(MODULES:=.o) $(GENERATED:=.o) 

HEADERS=

CLEAN_EXTRA=lex.yy.cc pddl+.cc pddl+.hh

LDFLAGS=$(FLEX_LIB) -lz 

CFLAGS=-O3 -DNWITH_CURSES -DPARSER_DEBUG -I. 


##CFLAGS=-ggdb -DNWITH_CURSES -DPARSER_DEBUG -I. -I/home/charlesg/downloads/boost_1_34_0/

#CFLAGS=-ggdb -DNATHAN -DPARSER_DEBUG -I. -I/home/charlesg/downloads/boost_1_34_0/

libayPlan: $(OBJECTS)
	ar rv libayPlan.so $(OBJECTS)
	ranlib libayPlan.so

all: $(GENERATED:=.cc) $(HEADERS) $(TARGET)

$(TARGET): $(OBJECTS) $(EXTRA_OBJECTS_EXECUTABLE)
	$(CXX) -o $@ $(EXTRA_OBJECTS_EXECUTABLE) $(OBJECTS) $(CFLAGS) $(LDFLAGS)

%.o: %.cc
	$(CXX) $(CFLAGS) $< -c -o $@

%.cc : %.yacc
	flex++ pddl+.lex ; bison $< -o $@

# %.cc: %.y
# 	bison -v -p$(patsubst %parser.y,%,$(patsubst %Parser.y,%,$<))yy -d  -o $@  $<
# 	mv $@.h $(patsubst %.cc,%,$@).hh 2>/dev/null || true

# %.cc: %.l
# 	flex -P$(patsubst %lexer.l,%,$(patsubst %Scanner.l,%,$<))yy -o$@ $<

clean:
	rm -f $(HEADERS) $(OBJECTS) $(TARGET) $(CLEAN_EXTRA)

test: $(TARGET)
	bash ./run-tests

