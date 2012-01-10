GARNAME = cogx-distro
GARVERSION =
CATEGORIES = distributions
DISTFILES =
MAINTAINER = mhanheid

DEB_PACKAGES=	libqt4-opengl-dev@ubuntu \
		libqt4-webkit@ubuntu \
		python-qt4@ubuntu \
		libboost-all-dev@ubuntu \
		libqt4-dev@ubuntu \
		libgl1-mesa-dev@ubuntu \
		libglu1-mesa-dev@ubuntu \
		libftgl-dev@ubuntu \
		liblua5.1-0-dev@ubuntu \
		libtolua++5.1-dev@ubuntu \
		libboost-serialization-dev@ubuntu \
		libboost-iostreams-dev@ubuntu \
		libqt4-dev@ubuntu \
		libtolua++5.1-dev@ubuntu \
		libprotobuf-dev@ubuntu \
		python-pygraphviz@ubuntu \
		libglew1.5-dev@ubuntu \
		libdevil-dev@ubuntu \
		nvidia-cg-toolkit@ubuntu \
		cmake-curses-gui@ubuntu \
		libdc1394-22-dev@ubuntu \
		libdc1394-utils@ubuntu \
		libsdl1.2-dev@ubuntu \
		libsuitesparse-dev@ubuntu\
		libeigen2-dev@ubuntu \
		python-numpy@ubuntu \
		flex@ubuntu \
		bison@ubuntu \
		libpcl-all@ubuntu \
		\
		qt4-mac-devel@macports \
		py-pyqt4@macports \
		boost@macports \
		mesa@macports \
		glut@macports \
		ftgl@macports \
		lua@macports \
		protobuf-cpp@macports \
		protobuf-java@macports \
		protobuf-c@macports \
		protobuf-python@macports \
		py26-pygraphviz@macports \
		glew@macports \
		libdevil@macports \
		libdc1394@macports \
		libsdl-devel@macports \
		SuiteSparse@macports\
		eigen@macports \
		eigen3@macports \
		py-numpy@macports \
		flex@macports \
		bison@macports 

LIBDEPS = 			contrib/peekabot \
		cogx/cast-c++ \
		cogx/cast-java \
		cogx/cosycure \
		contrib/opencv \
		contrib/player-3.0.2 \
		contrib/stage \
		contrib/mercury

DESCRIPTION = cogx-distro

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS = 

CONFIGURE_ARGS = 


libpcl-all@ubuntu:	add-pcl-ppa

add-pcl-ppa:
	add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
	apt-get update
	$(MAKECOOKIE)


include ../../gar.extend.mk

checksum-$(DISTFILES):	
	true
