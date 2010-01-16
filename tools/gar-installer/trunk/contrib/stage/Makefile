GARNAME = stage
GARVERSION = 2.1.1
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = mhanheid

MASTER_SITES = http://dfn.dl.sourceforge.net/sourceforge/playerstage/

LIBDEPS = contrib/player-2.1.2

DESCRIPTION = stage
define BLURB
  stagedef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS)

include ../../gar.mk

CONFIGURE_ARGS = $(DIRPATHS)  --with-color=/etc/X11/rgb.txt


