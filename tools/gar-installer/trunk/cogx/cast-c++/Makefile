GARNAME = cast
GARVERSION = 2.1.0
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = n.hawes

#MASTER_SITES = http://downloads.sourceforge.net/project/baltcast/cast%202/cast%202.1.0/
MASTER_SITES =  svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/latest/


LIBDEPS = 

DEB_PACKAGES=\
	liblog4cxx10-dev@ubuntu \
        libzeroc-ice33-dev@ubuntu \
        python-zeroc-ice@ubuntu

MFLAGS=$(MFLAGS_NUMCPU)


CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) -D OUTPUT:PATH=$(prefix) -D CMAKE_BUILD_TYPE=Debug

checksum-$(DISTFILES):  
	@true
checksum-$(PATCHFILES): 
	@true
	


	
	


