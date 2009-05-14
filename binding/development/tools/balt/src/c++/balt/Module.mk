OUTPUT=/home/nah/svn.cosy/code/tools/balt/output

VERSION=debug

PLATFORM=$(shell uname -s)

ifeq ("$(PLATFORM)","Darwin")
	SHARED=-dynamiclib
	JNI_LDFLAGS=-dynamiclib -framework JavaVM
	LIB_POSTFIX=dylib
	JNI_POSTFIX=jnilib
	CPPFLAGS=-I/System/Library/Frameworks/JavaVM.framework/Headers
	CC = g++
	OMNIIDL = omniidl
else
	SHARED=-shared -Wl,-O1 -Wl -z defs 
	JNI_LDFLAGS=-shared -Wl,-O1 -Wl -z defs 
	LIB_POSTFIX=so	
	JNI_POSTFIX=so		
	CPPFLAGS= -I$(JAVA_HOME)/include -I$(JAVA_HOME)/include/linux
	CC = g++
	OMNIIDL = omniidl
endif


