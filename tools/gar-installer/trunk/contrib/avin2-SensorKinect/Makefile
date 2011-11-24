GARNAME = avin2-SensorKinect
GARVERSION = 12ad25d
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).zip

MAINTAINER = ttoenige

LIBDEPS = contrib/openNI

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

include ../../gar.extend.mk

SYSTEM_UNAME:=$(shell uname -s)
ifeq ($(SYSTEM_UNAME),Darwin)
post-build:
	@echo "*** build not supported on $(SYSTEM_UNAME) ***" >&2
post-install:
	@echo "*** install not supported on $(SYSTEM_UNAME) ***" >&2	
else
post-build:
	(cd $(WORKSRC)/Platform/Linux-x86/CreateRedist/; bash ./RedistMaker;)
	@$(MAKECOOKIE)


post-install:
	(cd $(WORKSRC)/Platform/Linux-x86/Redist/; sudo bash ./install.sh -i;)
	@$(MAKECOOKIE)
endif
