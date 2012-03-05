GARNAME = cast
GARVERSION = 2.1.0
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = n.hawes

#PATCHFILES= gar-base.diff




#MASTER_SITES = http://downloads.sourceforge.net/project/baltcast/cast%202/cast%202.1.0/
MASTER_SITES =  svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/latest/
#MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/cast-2.1.16b/


LIBDEPS = 

DEB_PACKAGES=\
	liblog4cxx10-dev@ubuntu \
        zeroc-ice34@ubuntu \
	libgoogle-perftools-dev@ubuntu \
	cmake@ubuntu \
	\
	log4cxx@macports \
	google-perftools@macports \
	ice-cpp@macports \
	ice-python@macports \
	cmake@macports 
		

#MFLAGS=$(MFLAGS_NUMCPU)


CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) -D OUTPUT:PATH=$(prefix) -D CMAKE_BUILD_TYPE=Debug -D GOOGLE_PROFILER=On

checksum-$(DISTFILES):  
	@true
checksum-$(PATCHFILES): 
	@true
	


	
	


