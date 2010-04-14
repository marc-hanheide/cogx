GARNAME = peekabot
GARVERSION = 0.7.2
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.bz2
MAINTAINER = mhanheid

MASTER_SITES = http://heanet.dl.sourceforge.net/sourceforge/peekabot/

LIBDEPS = 

MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	libboost-dev@ubuntu libfltk1.1-dev@ubuntu libpng12-dev@ubuntu libxerces-c2-dev@ubuntu bzr@ubuntu libfreetype6-dev@ubuntu libglut3-dev@ubuntu libtool@ubuntu automake@ubuntu autoconf@ubuntu

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = $(DIRPATHS) 


