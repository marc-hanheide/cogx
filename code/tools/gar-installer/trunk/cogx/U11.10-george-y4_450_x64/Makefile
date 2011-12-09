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
		libsdl-dev@ubuntu \
		libsuitesparse-dev \
		libcminpack-dev \
		libplot-dev \
		libeigen2-dev \
		libeigen3-dev \
		flex@ubuntu \
		bison@ubuntu


SYSBITS=$(shell getconf LONG_BIT)

LIBDEPS = 	contrib/peekabot \
		cogx/cast-c++ \
		cogx/cast-java \
		contrib/opencv \
		cogx/cosycure \
		contrib/player-3.0.2 \
		contrib/avin2-SensorKinect \
		cogx/v4r \
		contrib/physx_x$(SYSBITS) \
		cogx/golem_katana300_x$(SYSBITS) \
		contrib/gazebo \
		contrib/mercury \
#		contrib/openNI \

DESCRIPTION = agai-distro

CONFIGURE_SCRIPTS = 
BUILD_SCRIPTS = 
INSTALL_SCRIPTS = 

CONFIGURE_ARGS = 

include $(GARDIR)/gar.extend.mk

checksum-$(DISTFILES):	
	true
