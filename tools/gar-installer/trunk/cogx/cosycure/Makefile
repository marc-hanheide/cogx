GARNAME = cosycure
GARVERSION = trunk
CATEGORIES = bootstrap
DISTFILES = $(GARNAME).tgz
MAINTAINER = mhanheid

MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/nah/cosycure/trunk/

#LIBDEPS = contrib/peekabot

DEB_PACKAGES=

#MFLAGS=$(MFLAGS_NUMCPU)


CONFIGURE_SCRIPTS = $(WORKSRC)/configure
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = 

checksum-$(DISTFILES):  
	true
checksum-$(PATCHFILES): 
	true
	
pre-configure: autoreconf

autoreconf:
	(cd $(WORKSRC); autoreconf --install)
	@$(MAKECOOKIE)

post-install: install-includes install-libs install-bins
	@$(MAKECOOKIE)
	
install-includes:
	(cp -Rf $(WORKSRC)/include/cure $(includedir))
	@$(MAKECOOKIE)
	
install-libs:
	(cp -Rf $(WORKSRC)/lib/cure $(libdir))
	@$(MAKECOOKIE)
	
install-bins:
	(cp -f $(WORKSRC)/bin/* $(bindir))
	@$(MAKECOOKIE)



