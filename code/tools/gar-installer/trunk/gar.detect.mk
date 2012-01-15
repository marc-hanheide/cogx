
LSB_RELEASE_BIN:=/usr/bin/lsb_release

SYSTEM_UNAME:=$(shell uname -s)
ifeq ($(SYSTEM_UNAME),Linux)
   SYSTEM_NAME:=$(shell $(LSB_RELEASE_BIN) -si)
   SYSTEM_VERSION:=$(shell $(LSB_RELEASE_BIN) -sr)
else
   SYSTEM_NAME:=$(SYSTEM_UNAME)
   SYSTEM_VERSION=x
endif
SYSTEM_RELEASE:=$(SYSTEM_NAME)-$(SYSTEM_VERSION)

SYSTEM_BITS:=$(shell getconf LONG_BIT)