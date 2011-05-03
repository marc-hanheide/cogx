GARNAME = cudatoolkit
GARVERSION = 2.3_linux_32_ubuntu9.04
CATEGORIES = bootstrap
MAINTAINER = hanheidm
DISTFILES = $(GARNAME)-$(GARVERSION).run


#CONFIGURE_SCRIPTS = $(WORKSRC)/configure
#BUILD_SCRIPTS = $(WORKSRC)/Makefile
#INSTALL_SCRIPTS = 

include ../../gar.mk

pre-extract:
	mkdir -p $(WORKSRC)/$(GARNAME)-$(GARVERSION)
	@$(MAKECOOKIE)

extract:
	cp $(DOWNLOADDIR)/$(DISTFILES) $(WORKSRC)
	@$(MAKECOOKIE)

post-install:
	sudo $(WORKSRC)/$(DISTFILES) -- --prefix=$(prefix)/cuda
	@$(MAKECOOKIE)

