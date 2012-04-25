GARDIR ?= ../..
GARNAME = libpcl
GARVERSION = 1.5
CATEGORIES = bootstrap
DISTFILES =
MAINTAINER = mmarko
DESCRIPTION = Point Cloud Library

include ../../gar.detect.mk

DEB_PACKAGES=	\
		libpcl-all-dev@ubuntu \

LIBDEPS = 

CONFIGURE_SCRIPTS =
BUILD_SCRIPTS =
INSTALL_SCRIPTS =
CONFIGURE_ARGS =

libpcl-all-dev@ubuntu:	add-pcl-ppa
APT_LIST_FILE = /etc/apt/sources.list.d/v-launchpad-jochen-sprickerhof-de-pcl-$(SYSTEM_CODENAME).list

add-pcl-ppa: $(APT_LIST_FILE)

$(APT_LIST_FILE):
	add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
	apt-get update
	$(MAKECOOKIE)

include ../../gar.extend.mk

