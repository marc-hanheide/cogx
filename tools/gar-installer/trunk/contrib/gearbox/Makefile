GARNAME = gearbox
GARVERSION = 9.11
CATEGORIES = bootstrap
MAINTAINER = ttoenige
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz

MASTER_SITES = http://downloads.sourceforge.net/project/gearbox/gearbox/${GARVERSION}/
PATCHFILES= gar-base.diff

# no parallel build due to problems of player make scheme
#MFLAGS=$(MFLAGS_NUMCPU)

#DEB_PACKAGES=

DESCRIPTION = player for cogx platform
define BLURB
  playerdef
endef

CONFIGURE_SCRIPTS = cmake
BUILD_SCRIPTS = $(WORKSRC)/Makefile
INSTALL_SCRIPTS = $(WORKSRC)/Makefile

CONFIGURE_ARGS = -D CMAKE_INSTALL_PREFIX=$(prefix) \
			-D LIBRARY_OUTPUT_PATH=$(prefix)/lib \
			-D ENABLE_LIB_BASICEXAMPLE=OFF \
			-D ENABLE_LIB_FLEXIPORT=ON \
			-D ENABLE_LIB_GBXADVANCEDEXAMPLE=OFF \
			-D ENABLE_LIB_GBXGARMINACFR=OFF \
			-D ENABLE_LIB_GBXICEUTILACFR=OFF \
			-D ENABLE_LIB_GBXNOVATELACFR=OFF \
			-D ENABLE_LIB_GBXSERIALACFR=OFF \
			-D ENABLE_LIB_GBXSICKACFR=OFF \
			-D ENABLE_LIB_GBXSMARTBATTERYACFR=OFF \
			-D ENABLE_LIB_GBXUTILACFR=OFF \
			-D HOKUYO_AIST_BUILD_BINDINGS=OFF
			-D ENABLE_LIB_HOKUYO_AIST=ON

include ../../gar.extend.mk

#pre-configure:
#	cd $(WORKSRC) && autoreconf -i
#	@$(MAKECOOKIE)


pre-install:
	if [ ! -e /usr/local/lib641 ]; then ln -s /usr/local/lib /usr/local/lib64; fi
