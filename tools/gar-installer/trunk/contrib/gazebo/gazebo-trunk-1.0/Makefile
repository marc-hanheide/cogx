GARDIR    ?= ../../..
GARNAME    = gazebo
GARVERSION = rostrunk
CATEGORIES = bootstrap
MAINTAINER = mmarko
DISTNAME   = $(GARNAME)-$(GARVERSION)
DISTFILES  = $(DISTNAME).tar.gz

MASTER_SITES = hghttps://kforge.ros.org/gazebo/hg/
# Select the revision:
#  * dev     : the latest on development branch
#  * default : the latest on default branch
#  * -r NNN  : select a particular revision
HG_REVISION = -r 2707
PATCHFILES   = \
	$(DISTNAME)-2707-cogx.patch \
	$(DISTNAME)-2707-cogx.a.patch


include $(GARDIR)/gar.detect.mk

DEB_PACKAGES=\
	mercurial@ubuntu \
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
#checksum-$(DISTNAME)-2707-cogx.patch: 
#        true
#checksum-$(DISTNAME)-2707-cogx.a.patch: 
#        true

