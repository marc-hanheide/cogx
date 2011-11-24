GARNAME = v4r
GARVERSION = 1.0
CATEGORIES = bootstrap
#DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = ari

#PATCHFILES= gar-base.diff

DEB_PACKAGES=	libglew1.5-dev@ubuntu \
		libdevil-dev@ubuntu

#MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/cast-2.1.16b/
MASTER_SITES = svnhttps://repo.acin.tuwien.ac.at/v4r/stable/castv4r/

LIBDEPS = 

#DEB_PACKAGES=\
#	liblog4cxx10-dev@ubuntu 

MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) -D OUTPUT:PATH=$(prefix) -D CMAKE_BUILD_TYPE=Debug -D GOOGLE_PROFILER=On

checksum-$(DISTFILES):  
	@true
checksum-$(PATCHFILES): 
	@true
	


	
	


