GARNAME = NITE-Bin-Linux32
GARVERSION = v1.4.1.2
CATEGORIES = bootstrap
DISTNAME = $(GARNAME)-$(GARVERSION)
DISTFILES = $(DISTNAME).tar.bz2

MAINTAINER = hanheidm

LIBDEPS = contrib/openNI

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS = 




include ../../gar.extend.mk


# key provided by http://www.openni.org/downloadfiles/opennimodules

post-install:
	(cd $(WORKDIR); sudo ./install.sh -l=0KOIk2JeIBYClPWVnMoRKn5cdY4=;)
	@$(MAKECOOKIE)

