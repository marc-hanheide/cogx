GARNAME = v4r
GARVERSION = 1.0
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = ari

ifneq ($(CASTV4RPASS),)
SVNUSER=cogx --password $(CASTV4RPASS)
endif


MASTER_SITES = svnhttps://repo.acin.tuwien.ac.at/v4r/stable/castv4r/


DEB_PACKAGES=	libglew1.5-dev@ubuntu \
		libdevil-dev@ubuntu \
		libpcl-all-dev@ubuntu \
		libcminpack-dev@ubuntu

LIBDEPS = 

#MFLAGS=$(MFLAGS_NUMCPU)

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

#libpcl-all-dev@ubuntu:	add-pcl-ppa

#add-pcl-ppa:
#        add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
#        apt-get update
#        $(MAKECOOKIE)

include ../../gar.extend.mk

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX:PATH=$(prefix) -D OUTPUT:PATH=$(prefix) -D CMAKE_BUILD_TYPE=Debug -D GOOGLE_PROFILER=On

