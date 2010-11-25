GARNAME = golem
GARVERSION = trunk
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz
WORKSRC = $(WORKDIR)/$(DISTNAME)/compiler/cmake/default/

MAINTAINER = ttoenige


MASTER_SITES =

LIBDEPS = cogx/kni_3.9.2

DEB_PACKAGES= 

MFLAGS=$(MFLAGS_NUMCPU)


CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile


include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix)

checksum-$(DISTFILES):  
	true
checksum-$(PATCHFILES): 
	true

post-install:
	cp $(WORKDIR)/$(DISTNAME)/resources 
