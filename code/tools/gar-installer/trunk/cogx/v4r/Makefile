GARNAME = v4r
GARVERSION = 1.0
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = ari

#PATCHFILES= gar-base.diff

DEB_PACKAGES=	libglew1.5-dev@ubuntu \
		libdevil-dev@ubuntu \
		libcminpack-dev@ubuntu

LIBDEPS = 

MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) -D OUTPUT:PATH=$(prefix) -D CMAKE_BUILD_TYPE=Debug -D GOOGLE_PROFILER=On

