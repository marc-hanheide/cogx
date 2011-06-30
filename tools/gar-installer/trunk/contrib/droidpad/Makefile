GARNAME = droidpad
GARVERSION = 1.1
CATEGORIES = bootstrap
MAINTAINER = mhanheid
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MASTER_SITES = http://launchpad.net/droidpad-linux/1.0/1.1/+download/

#PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	intltool@ubuntu \
	libgtkmm-2.4-dev@ubuntu


DESCRIPTION = player for cogx platform
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS) 

include ../../gar.extend.mk

#pre-configure:
#	cd $(WORKSRC) && autoreconf -i
#	@$(MAKECOOKIE)

