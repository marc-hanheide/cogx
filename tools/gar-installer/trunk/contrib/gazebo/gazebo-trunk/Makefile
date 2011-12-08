GARNAME = gazebo
GARVERSION = trunk
CATEGORIES = bootstrap
MAINTAINER = mmarko
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.gz
#DISTFILES = $(GARNAME)-$(GARVERSION).tar.bz2

#MASTER_SITES = svnhttps://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/branches/wg
#MASTER_SITES = http://downloads.sourceforge.net/project/playerstage/Gazebo/$(GARVERSION)/
MASTER_SITES = svnhttps://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk/
SVN_REVISION=-r 9081

PATCHFILES= gazebo-trunk-r9081.diff

DEB_PACKAGES=\
	libode1sp@ubuntu libode-sp-dev@ubuntu libogre-dev@ubuntu ogre-tools@ubuntu libxml2-dev@ubuntu

LIBDEPS = contrib/player-3.0.2

DESCRIPTION = gazebo for cogx platform
define BLURB
  gazebodef
endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix)

include ../../gar.extend.mk


makesum-$(DISTFILES):  
	true
checksum-$(DISTFILES):  
	true
#checksum-$(PATCHFILES): 
#	true

