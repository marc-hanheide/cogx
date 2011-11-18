GARNAME = golem
GARVERSION = trunk
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz
WORKSRC = $(WORKDIR)/$(DISTNAME)/compiler/cmake/default/

MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/msk/development/Golem/trunk/

PATCHDIR=$(WORKDIR)/$(DISTNAME)
PATCHFILES = gar-base.diff

#LIBDEPS = contrib/peak-linux

MAINTAINER = ttoenige


CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile


include ../../gar.extend.mk

SYSBITS=$(shell getconf LONG_BIT)
OPT_BUILD_32=ERROR/SYSBITS
ifeq ($(SYSBITS),64)
    OPT_BUILD_32=OFF
endif
ifeq ($(SYSBITS),32)
    OPT_BUILD_32=ON
endif
CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) \
			-D BUILD_DEVICE_KATANA450=OFF \
			-D BUILD_AXNI=OFF \
			-D BUILD_32_BIT=$(OPT_BUILD_32)

mydebugprint:
	@echo $(CONFIGURE_ARGS)
