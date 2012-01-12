GARDIR ?= ../..
include $(GARDIR)/gar.mk

LSB_RELEASE_BIN:=/usr/bin/lsb_release


ifdef SVNUSER
SVNUSERARG=--username $(SVNUSER)
else
SVNUSERARG=
endif

pre-configure:	ubuntu-install

ubuntu-install:	$(DEB_PACKAGES)


print-debs:
	@echo -n "$(DEB_PACKAGES) "

%@ubuntu:
	@(if [ -x $(LSB_RELEASE_BIN) ]; then if [ `$(LSB_RELEASE_BIN) -i -s`x = "Ubuntu"x ]; then if dpkg-query -s $* | grep "Status:.*install[^e]" >/dev/null 2>&1; then \
		echo "	[ubuntu-check] $* is already installed.";\
		exit 0;\
	else \
		echo "	[ubuntu-check] need to install $*. will now call sudo apt-get install $*.";\
		sudo apt-get --allow-unauthenticated --assume-yes install $* || (echo "$* failed. press [return] to continue anyway or CTRL-C to interrupt."; read dummy);\
	fi;fi;fi) && $(MAKECOOKIE)


%@macports:
	@(if [ `uname  -s`x = "Darwin"x ] && which port; then if port installed $* | grep -v -q "None of"  >/dev/null 2>&1; then \
		echo "	[macports-check] $* is already installed.";\
		exit 0;\
	else \
		echo "	[macports-check] need to install $*. will now call sudo port install $*.";\
		sudo port install $* configure.compiler=gcc || (echo "$* failed. press [return] to continue anyway or CTRL-C to interrupt."; read dummy);\
	fi;fi) && $(MAKECOOKIE)


configure-cmake:
	@echo " ==> Running cmake in $(WORKSRC)"
	@cd $(WORKSRC) && $(CONFIGURE_ENV) cmake $(CONFIGURE_ARGS) .
	@$(MAKECOOKIE)

# build from a standard gnu-style makefile's default rule.
build-%/ant:
	@echo " ==> Running ant in $*"
	(cd $(WORKSRC) &&  $(BUILD_ENV) ant $(BUILD_ARGS))
	@$(MAKECOOKIE)

install-%/ant:
	@echo " ==> Running ant to install in $*"
	(cd $(WORKSRC) &&  $(INSTALL_ENV) ant $(INSTALL_ARGS))
	@$(MAKECOOKIE)

#build-ant:
#	@echo " ==> Running make in $*"
#	(cd $(WORKSRC) &&  ant )
#	@$(MAKECOOKIE)


# $(PARTIALDIR)/%-svn.tgz:
# 	(if [ ! -f "$(DOWNLOADDIR)/$(DISTFILES)" ]; then \
# 		rm -rf /tmp/gar-icewing/*;\
# 		mkdir -p /tmp/gar-icewing/svn;\
# 		cd /tmp/gar-icewing/svn; \
# 		echo "exporting from SVN: svn export $(SVN_REVISION) $(SVN_PATH)";\
# 		svn export $(SVN_REVISION) $(SVN_PATH); \
# 		mv trunk $(GARNAME)-$(GARVERSION);\
# 		tar czv -C /tmp/gar-icewing/svn -f /tmp/gar-icewing/$(DISTFILES) . ;\
# 	fi) 



/tmp/gartmp-$(USER)-$(GARNAME):
	mkdir -p $@

svnhttps//%:	/tmp/gartmp-$(USER)-$(GARNAME)
	(cd $< && \
	 rm -rf * && \
	 echo "exporting from SVN: svn export $(SVN_REVISION) https://$(*D)" && \
	 svn export $(SVNUSERARG) $(SVN_REVISION) https://$(*D) && \
	 mv * $(GARNAME)-$(GARVERSION)\
	)
	tar czv -C $< -f $(PARTIALDIR)/$(DISTFILES) .

svnhttp//%:	/tmp/gartmp-$(USER)-$(GARNAME)
	(cd $< && \
	 echo "exporting from SVN: svn export $(SVN_REVISION) http://$(*D)" && \
	 svn export  $(SVNUSERARG) $(SVN_REVISION) http://$(*D) && \
	 mv * $(GARNAME)-$(GARVERSION)\
	)
	tar czv -C $< -f $(PARTIALDIR)/$(DISTFILES) .

svn//%:	/tmp/gartmp-$(USER)-$(GARNAME)
	(cd $< && \
	 echo "exporting from SVN: svn export $(SVN_REVISION) file:///$(*D)" && \
	 svn export  $(SVNUSERARG) $(SVN_REVISION) file:///$(*D) && \
	 mv * $(GARNAME)-$(GARVERSION)\
	)
	tar czv -C $< -f $(PARTIALDIR)/$(DISTFILES) .
	@$(MAKE) makesum



