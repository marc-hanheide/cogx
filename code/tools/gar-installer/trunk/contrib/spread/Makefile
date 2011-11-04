GARNAME = spread-src
GARVERSION = 4.1.0
CATEGORIES = bootstrap
MAINTAINER = mhanheid
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

#MASTER_SITES = http://downloads.sourceforge.net/project/playerstage/Player/${GARVERSION}/
#PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

DEB_PACKAGES=\
	autotools-dev@ubuntu build-essential@ubuntu cmake@ubuntu cpp@ubuntu 

LIBDEPS = 


DESCRIPTION = player for cogx platform
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile  
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = $(DIRPATHS) 



include ../../gar.extend.mk

post-build:
	cd $(WORKSRC)/java && ant
	@$(MAKECOOKIE)
	
post-install:
	cd $(WORKSRC)/java && ant -Dlib.dir=$(libdir)/java install
	@$(MAKECOOKIE)
	
	
#pre-configure:
#	cd $(WORKSRC) && autoreconf -i
#	@$(MAKECOOKIE)

