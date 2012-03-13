GARNAME = Stage
GARVERSION = 3.2.2-Source
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

#PATCHFILES = macosx-fltk-patch.diff

MAINTAINER = mhanheid

PATCHFILES= gar-base.diff ros-fltk-patch.diff macosx-fltk-patch.diff stage-3.2.2-consumequeue-20100615.patch

MASTER_SITES = http://dfn.dl.sourceforge.net/sourceforge/playerstage/

LIBDEPS = contrib/player-3.0.2

MFLAGS=-j4#$(MFLAGS_NUMCPU)

DESCRIPTION = stage
define BLURB
  stagedef
endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

SYSTEM_UNAME:=$(shell uname -s)
ifeq ($(SYSTEM_UNAME),Darwin)
	CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)
else
	CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix) -D CMAKE_CXX_FLAGS="-Wl,--no-as-needed"
endif



include ../../gar.extend.mk



