GARNAME = mercury-compiler
GARVERSION = 10.04.1
CATEGORIES = bootstrap
MAINTAINER = fsiepman
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MASTER_SITES = http://www.mercury.csse.unimelb.edu.au/download/files/
PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	bison@ubuntu flex@ubuntu

DESCRIPTION = mercury
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS) --enable-libgrades=hlc.gc,hlc.gc.prof,hlc.gc.memprof,hlc.gc.tr,hlc.par.gc

include ../../gar.extend.mk

