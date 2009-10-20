CXXFLAGS=-Wall
LDFLAGS=

BIN=abducer-server-mk2

HEADERS=Tokens.h TermTokeniser.h TermParsing.h Terms.h

CXX_SOURCES=Tokens.cc TermTokeniser.cc ServerMain.cc TermParsing.cc Terms.cc

OBJS=$(addsuffix .o, $(basename $(CXX_SOURCES))) \

$(BIN): $(HEADERS) $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

%.o: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

.PHONY: clean
clean:
	rm -f $(OBJS) $(BIN)
