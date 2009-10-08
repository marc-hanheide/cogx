GARNAME = mercury-compiler
GARVERSION = rotd-2009-10-05
CATEGORIES = bootstrap
MAINTAINER = fsiepman
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MASTER_SITES = http://www.mercury.cs.mu.oz.au/download/files/beta-releases/rotd/
#PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

DESCRIPTION = mercury
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS) --enable-libgrades=hlc.gc,hlc.gc.prof,hlc.gc.memprof,hlc.gc.tr,hlc.par.gc

include ../../gar.mk

