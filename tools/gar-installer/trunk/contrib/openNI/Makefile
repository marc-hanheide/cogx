GARNAME = OpenNI-OpenNI
GARVERSION = d50d0e3
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz

MAINTAINER = ttoenige

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

include ../../gar.extend.mk


post-build:
	(cd $(WORKSRC)/Platform/Linux-x86/CreateRedist/; ./RedistMaker;)
	@$(MAKECOOKIE)


post-install:
	(cd $(WORKSRC)/Platform/Linux-x86/Redist/; sudo ./install.sh -i $(prefix);)
	@$(MAKECOOKIE)

