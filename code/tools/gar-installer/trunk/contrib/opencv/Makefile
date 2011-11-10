GARNAME = OpenCV
GARVERSION = 2.3.1
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION)a.tar.bz2
MAINTAINER = ttoenige

MASTER_SITES =  http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.3.1/
#svnhttps://opencvlibrary.svn.sourceforge.net/svnroot/opencvlibrary/trunk/opencv/



LIBDEPS = 

#DEB_PACKAGES=\
#        libdc1394-22-dev@ubuntu \
#        libdc1394-utils@ubuntu \
#        cmake@ubuntu \
#        libgtk2.0-dev@ubuntu
# libv4l-dev

MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk
#-D OPENCV_CONFIG_FILE_INCLUDE_DIR:PATH=$(prefix)/include/opencv
CONFIGURE_ARGS =  -D CMAKE_INSTALL_PREFIX:PATH=$(prefix)



#checksum-$(DISTFILES):  
#	true
#checksum-$(PATCHFILES): 
#	true

#SVN_REVISION=-r 2218





