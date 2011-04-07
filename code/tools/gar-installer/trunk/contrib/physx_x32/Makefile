GARNAME = PhysX_x32
GARVERSION = 2.8.3.3
CATEGORIES = bootstrap
MAINTAINER = ttoenige
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

DESCRIPTION = physx
define BLURB
  physxdef
endef

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS =

post-install:
	dpkg -i $(WORKSRC)/libphysx-common-2.8.3_3_i386.deb
	dpkg -i $(WORKSRC)/libphysx-2.8.3_3_i386.deb
	dpkg -i $(WORKSRC)/libphysx-dev-2.8.3_3_i386.deb
	dpkg -i $(WORKSRC)/libphysx-doc-2.8.3_3_i386.deb
	dpkg -i $(WORKSRC)/libphysx-extras-2.8.3_3_i386.deb
	dpkg -i $(WORKSRC)/libphysx-samples-2.8.3_3_i386.deb


include ../../gar.extend.mk

