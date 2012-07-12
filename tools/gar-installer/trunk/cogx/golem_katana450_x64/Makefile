GARNAME = golem
GARVERSION = trunk
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz
WORKSRC = $(WORKDIR)/$(DISTNAME)/compiler/cmake/default/

PATCHDIR=$(WORKDIR)/$(DISTNAME)
PATCHFILES = golem-cogx-1.patch

LIBDEPS = contrib/peak-linux

MAINTAINER = ttoenige


CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile


include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix) \
			-D BUILD_DEVICE_KATANA300=OFF \
			-D BUILD_32_BIT=OFF

