GARDIR ?= ../../..
GARNAME = gazebo
GARVERSION = trunk-r9081-patched
CATEGORIES = bootstrap
MAINTAINER = mmarko
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.bz2
#DISTFILES = $(GARNAME)-$(GARVERSION).tar.bz2

#The old gazebo repository that was removed
#MASTER_SITES = svnhttps://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk/
#SVN_REVISION=-r 9081
#PATCHFILES= gazebo-trunk-r9081.diff

MASTER_SITES = http://vicos.fri.uni-lj.si/data/cogx/gazebo/

include $(GARDIR)/gar.detect.mk

DEB_PACKAGES=\
	libode1sp@ubuntu \
	libode-sp-dev@ubuntu \
	libogre-dev@ubuntu \
	ogre-tools@ubuntu \
	libyaml-dev@ubuntu \
	libxml2-dev@ubuntu

ifeq ("$(SYSTEM_RELEASE)","Ubuntu-11.10")
   DEB_PACKAGES+= \
	libwxgtk2.8-dev@ubuntu
endif

LIBDEPS = contrib/player-3.0.2

DESCRIPTION = gazebo for cogx platform
define BLURB
  gazebodef
endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)

include $(GARDIR)/gar.extend.mk


#makesum-$(DISTFILES):  
#        true
#checksum-$(DISTFILES):  
#        true
#checksum-$(PATCHFILES): 
#	true

