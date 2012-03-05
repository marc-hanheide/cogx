GARNAME = cast
GARVERSION = svn
CATEGORIES = bootstrap
DISTFILES = $(GARNAME)-$(GARVERSION).tar.gz
MAINTAINER = n.hawes

#MASTER_SITES = http://downloads.sourceforge.net/project/baltcast/cast%202/cast%202.1.0/

MASTER_SITES =  svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/latest/
#MASTER_SITES = svnhttps://codex.cs.bham.ac.uk/svn/nah/cast/tags/cast-2.1.16b/

#MASTER_SITES =  svnhttps://codex.cs.bham.ac.uk/svn/nah/cosy/development/cast/tags/cast-2.1.0rc3/


LIBDEPS = cogx/cast-c++

DEB_PACKAGES=\
	liblog4j1.2-java@ubuntu \
	default-jdk@ubuntu \
        ant@ubuntu \
        zeroc-ice34@ubuntu \
    \
	jakarta-log4j@macports \
	apache-ant@macports \
	ice-java@macports

	
#MFLAGS=$(MFLAGS_NUMCPU)


CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = $(WORKSRC)/ant
INSTALL_SCRIPTS = $(WORKSRC)/ant

BUILD_ENV += CLASSPATH=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar:/usr/share/java/log4j-1.2.jar:
INSTALL_ENV += CLASSPATH=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar:/usr/share/java/log4j-1.2.jar:



include ../../gar.extend.mk


CONFIGURE_ARGS = -Dinstall.prefix=$(prefix) -Dinstall.dir=$(prefix)/share/java -Dclasspath=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar
BUILD_ARGS = $(CONFIGURE_ARGS) compile
INSTALL_ARGS = $(CONFIGURE_ARGS) install


checksum-$(DISTFILES):  
	@true
checksum-$(PATCHFILES): 
	@true
	


	
	


