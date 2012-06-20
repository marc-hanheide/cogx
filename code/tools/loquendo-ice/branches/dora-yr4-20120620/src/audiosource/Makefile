#------------------------------------------------------------------------------#

LIBNAME=libaudiosource.a

CXX_HEADERS+=AudioSource.h PCMCapture.h PCMCaptureAudioSource.h RAWFileAudioSource.h
CXX_SOURCES+=AudioSource.cc PCMCapture.cc PCMCaptureAudioSource.cc RAWFileAudioSource.cc

CXXFLAGS+=-Wall
CXXFLAGS+=-O2
CXXFLAGS+=-g
CXXFLAGS+=-I../utils
CXXFLAGS+=-I$(LASR_INSTALL_PATH)/includes
CXXFLAGS+=$(shell pkg-config --cflags libpulse-simple)

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

%.o: %.cc $(CXX_HEADERS)
	$(CXX) $(CXXFLAGS) -c $*.cc

$(LIBNAME): $(CXX_HEADERS) $(OBJS)
	ar -crs $@ $(OBJS)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	rm -rf $(OBJS) \
		$(LIBNAME)
