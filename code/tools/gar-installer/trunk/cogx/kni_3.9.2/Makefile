GARNAME = KNI
GARVERSION = 3.9.2
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz
WORKSRC = $(WORKDIR)/$(DISTNAME)/compiler/cmake/default/

MAINTAINER = ttoenige

SVNUSER = pub
MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/msk/development/KNI_v3.9.2/trunk/

LIBDEPS =

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

