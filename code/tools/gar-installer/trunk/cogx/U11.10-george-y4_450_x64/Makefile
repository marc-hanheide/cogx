GARDIR ?= ../..
GARNAME = cogx-distro
GARVERSION =
CATEGORIES = distributions
DISTFILES =
MAINTAINER = ari

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
		cmake-qt-gui@ubuntu \
		libdc1394-22-dev@ubuntu \
		libdc1394-utils@ubuntu \
		libsdl-dev@ubuntu \
		libsuitesparse-dev@ubuntu \
		libcminpack-dev@ubuntu \
		libplot-dev@ubuntu \
		libeigen2-dev@ubuntu \
		libeigen3-dev@ubuntu \
		flex@ubuntu \
		bison@ubuntu


SYSBITS=$(shell getconf LONG_BIT)

LIBDEPS = 	contrib/peekabot \
		cogx/cast-c++ \
		cogx/cast-java \
		cogx/cosycure \
		contrib/opencv \
		contrib/player-3.0.2 \
		cogx/v4r \
		contrib/physx_x$(SYSBITS) \
		cogx/golem_katana300_x$(SYSBITS) \
		contrib/mercury 
#		contrib/avin2-SensorKinect \
#		contrib/gazebo \
#		contrib/openNI \

DESCRIPTION = agai-distro

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS = 

CONFIGURE_ARGS = 

include $(GARDIR)/gar.extend.mk

checksum-$(DISTFILES):	
	true
