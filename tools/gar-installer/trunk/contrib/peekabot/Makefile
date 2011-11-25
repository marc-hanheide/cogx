GARNAME = peekabot
GARVERSION = 0.8.6
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.bz2
MAINTAINER = mhanheid


SYSTEM_UNAME=$(shell uname -s)

MASTER_SITES = http://downloads.sourceforge.net/project/peekabot/peekabot/0.8.x/

LIBDEPS = 

PATCHFILES= gar-base.diff

#MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	libboost-all-dev@ubuntu libfltk1.1-dev@ubuntu libpng12-dev@ubuntu libxerces-c2-dev@ubuntu bzr@ubuntu libfreetype6-dev@ubuntu libtool@ubuntu automake@ubuntu autoconf@ubuntu libgtkglextmm-x11-1.2-dev@ubuntu libgtkmm-2.4-dev@ubuntu \
		boost@macports fltk-devel@macports libpng@macports xercesc@macports bzr@macports freetype@macports libtool@macports automake@macports autoconf@macports gtk2@macports gtkglextmm@macports  gtkmm@@macports


CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = $(DIRPATHS) 


pre-configure: reconf showconfig

showconfig:
	@echo "configure for $(SYSTEM_UNAME): $(CONFIGURE_ARGS)"
	$(MAKECOOKIE)


reconf:
	cd $(WORKSRC) && autoreconf -i
	$(MAKECOOKIE)

