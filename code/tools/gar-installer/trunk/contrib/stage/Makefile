GARNAME = Stage
GARVERSION = 3.2.2-Source
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MAINTAINER = mhanheid

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

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)

include ../../gar.extend.mk

CONFIGURE_ARGS = $(DIRPATHS)  --with-color=/etc/X11/rgb.txt


