GARNAME = assimp-
GARVERSION = 2.0.863-sdk
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).zip
MAINTAINER = mmarko

MASTER_SITES = http://sourceforge.net/projects/assimp/files/assimp-2.0/
#PATCHFILES=pixeltype-bug.diff

LIBDEPS = 

#DEB_PACKAGES=\
#        libdc1394-22-dev@ubuntu \
#        libdc1394-utils@ubuntu \
#        cmake@ubuntu \
#        libgtk2.0-dev@ubuntu
# libv4l-dev

#MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk
#CONFIGURE_ARGS =  -D CMAKE_INSTALL_PREFIX:PATH=$(prefix)



#checksum-$(DISTFILES):  
#	true
#checksum-$(PATCHFILES): 
#	true

#SVN_REVISION=-r 2218





