#------------------------------------------------------------------------------#

LIBNAME=libtty.a

CXX_HEADERS=TtyUtils.h
CXX_SOURCES=TtyUtils.cc

CXXFLAGS=-Wall
CXXFLAGS+=-I. $(ICE_INCLUDE)
CXXFLAGS+=-g
CXXFLAGS+=-O2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: main
main: $(LIBNAME)

#------------------------------------------------------------------------------#
# You shouldn't need to edit anything below.
#------------------------------------------------------------------------------#

ifeq ($(origin DEBUG), undefined)
else
CXXFLAGS+=-DDEBUG
endif

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

OBJS=$(addsuffix .o, $(basename $(CXX_SOURCES)))

.PRECIOUS: %.o
%.o: %.cc $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

lib%.a: $(CXX_HEADERS) $(OBJS)
	ar -crs $@ $(OBJS)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	rm -rf $(OBJS) \
		$(LIBNAME)
