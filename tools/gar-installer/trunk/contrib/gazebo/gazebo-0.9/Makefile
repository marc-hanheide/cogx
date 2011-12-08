GARDIR ?= ../../..
GARNAME = gazebo
GARVERSION = 0.9.0
CATEGORIES = bootstrap
MAINTAINER = ttoenige
DISTFILES = $(GARNAME)-$(GARVERSION).tar.bz2

#MASTER_SITES = svnhttps://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/branches/wg
#MASTER_SITES = svnhttps://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk/
MASTER_SITES = http://downloads.sourceforge.net/project/playerstage/Gazebo/$(GARVERSION)/

PATCHFILES= gar-base.diff gazebo-initialisation.diff ogre-visual-name.diff gazeborc-noinstall.diff

DEB_PACKAGES=\
	libode1sp@ubuntu libode-sp-dev@ubuntu libogre-dev@ubuntu ogre-tools@ubuntu libxml2-dev@ubuntu
# ubuntu 11.04: libogremain-1.6.4@ubuntu 

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


#checksum-$(DISTFILES):  
#	true
#checksum-$(PATCHFILES): 
#	true

