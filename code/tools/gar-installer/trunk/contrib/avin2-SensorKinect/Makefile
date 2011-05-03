GARNAME = avin2-SensorKinect
GARVERSION = 0124bd2
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz

MAINTAINER = ttoenige

LIBDEPS = contrib/openNI

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

include ../../gar.extend.mk


post-build:
	(cd $(WORKSRC)/Platform/Linux-x86/CreateRedist/; ./RedistMaker;)
	@$(MAKECOOKIE)


post-install:
	(cd $(WORKSRC)/Platform/Linux-x86/Redist/; sudo ./install.sh -i;)
	@$(MAKECOOKIE)

