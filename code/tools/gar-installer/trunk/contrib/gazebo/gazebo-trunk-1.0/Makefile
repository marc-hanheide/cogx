GARDIR    ?= ../../..
GARNAME    = gazebo
GARVERSION = rostrunk
CATEGORIES = bootstrap
MAINTAINER = mmarko
DISTNAME   = $(GARNAME)-$(GARVERSION)
DISTFILES  = $(DISTNAME).tar.gz

MASTER_SITES = hghttps://kforge.ros.org/gazebo/hg/
# XXX HG_REVISION = -r 2717
PATCHFILES   = $(DISTNAME)-cogx.patch

include $(GARDIR)/gar.detect.mk

DEB_PACKAGES=\
	libtinyxml-dev@ubuntu \
	ogre-tools@ubuntu \
	libogre-dev@ubuntu
#        libode1sp@ubuntu \
#        libode-sp-dev@ubuntu \
#        libyaml-dev@ubuntu \
#        libxml2-dev@ubuntu

#ifeq ("$(SYSTEM_RELEASE)","Ubuntu-11.10")
#   DEB_PACKAGES+= \
#        libwxgtk2.8-dev@ubuntu
#endif

LIBDEPS = contrib/player-3.0.2

DESCRIPTION = Gazebo for cogx platform
#define BLURB
#  gazebodef
#endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)

include $(GARDIR)/gar.extend.mk


makesum-$(DISTFILES):  
	true
checksum-$(DISTFILES):  
	true
checksum-$(PATCHFILES): 
	true

