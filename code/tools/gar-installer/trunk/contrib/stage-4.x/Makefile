GARNAME = Stage
GARVERSION = 4.0.0
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)-src
DISTFILES = $(DISTNAME).tar.gz
MAINTAINER = ttoenige

MASTER_SITES = http://cloud.github.com/downloads/rtv/Stage/

WORKSRC ?= $(WORKDIR)/$(DISTNAME)

LIBDEPS = contrib/player-trunk

DESCRIPTION = stage
define BLURB
  stagedef
endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)

include ../../gar.extend.mk

#CONFIGURE_ARGS = $(DIRPATHS)  --with-color=/etc/X11/rgb.txt


