GARNAME = player
GARVERSION = 2.1.3
CATEGORIES = bootstrap
MAINTAINER = fsiepman
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MASTER_SITES = http://dfn.dl.sourceforge.net/sourceforge/playerstage/

PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	libboost-all-dev@ubuntu libfltk1.1-dev@ubuntu libpng12-dev@ubuntu libxerces-c2-dev@ubuntu libfreetype6-dev@ubuntu libglut3-dev@ubuntu libtool@ubuntu automake@ubuntu autoconf@ubuntu


DESCRIPTION = player for cogx platform
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS) --disable-sicklms400 --disable-mricp --with-boost-signals=mt --with-boost-thread=mt

include ../../gar.extend.mk

pre-configure:
	cd $(WORKSRC) && autoreconf -i
	@$(MAKECOOKIE)

