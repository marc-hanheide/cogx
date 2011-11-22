GARNAME = OpenNI-OpenNI
GARVERSION = d50d0e3
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz

MAINTAINER = ttoenige

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

DEB_PACKAGES=\
	libusb-1.0-0-dev@ubuntu \
	libmono-winforms2.0-cil@ubuntu



include ../../gar.extend.mk

SYSTEM_UNAME:=$(shell uname -s)
ifeq ($(SYSTEM_UNAME),Darwin)
post-build:
	@echo "*** build not supported on $(SYSTEM_UNAME) ***" >&2
post-install:
	@echo "*** install not supported on $(SYSTEM_UNAME) ***" >&2	
else
post-build:
	(cd $(WORKSRC)/Platform/Linux-x86/CreateRedist/; ./RedistMaker;)
	@$(MAKECOOKIE)


post-install:
	(cd $(WORKSRC)/Platform/Linux-x86/Redist/; sudo ./install.sh -i;)
	@$(MAKECOOKIE)
endif

