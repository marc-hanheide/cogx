GARNAME = OpenNI-OpenNI
GARVERSION = 	d07a88a
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).zip

MAINTAINER = ttoenige

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

DEB_PACKAGES=\
	libusb-1.0-0-dev@ubuntu \
	libmono-winforms2.0-cil@ubuntu

PATCHFILES=memcpy-bugfix.diff

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

