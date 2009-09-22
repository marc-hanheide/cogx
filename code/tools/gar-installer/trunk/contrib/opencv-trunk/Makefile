GARNAME = opencv
GARVERSION = trunk
CATEGORIES = bootstrap
DISTFILES = $(GARNAME).tgz
MAINTAINER = mhanheid

MASTER_SITES =  svnhttps://opencvlibrary.svn.sourceforge.net/svnroot/opencvlibrary/trunk/opencv/


LIBDEPS = 

DEB_PACKAGES=\
        libdc1394-22-dev@ubuntu \
        libdc1394-utils@ubuntu

MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = -D OPENCV_CONFIG_FILE_INCLUDE_DIR:PATH=$(prefix)/include/opencv -D CMAKE_INSTALL_PREFIX:PATH=$(prefix)

checksum-$(DISTFILES):  
	true
checksum-$(PATCHFILES): 
	true
	
SVN_REVISION=-r 1845

	
	


