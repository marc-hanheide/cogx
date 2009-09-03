#-*- mode: Fundamental; tab-width: 4; -*-
# ex:ts=4
# $Id: gar.conf.mk.sample,v 1.7 2005/08/17 13:59:45 swrede Exp $

# This file contains configuration variables that are global to
# the GAR system.  Users wishing to make a change on a
# per-package basis should edit the category/package/Makefile, or
# specify environment variables on the make command-line.

# Variables that define the default *actions* (rather than just
# default data) of the system will remain in bbc.gar.mk
# (bbc.port.mk)

# Setting this variable will cause the results of your builds to
# be cleaned out after being installed.  Uncomment only if you
# desire this behavior!

# export BUILD_CLEAN = true

# The GARCHIVEDIR is a directory containing cached files. It can be created
# manually, or with 'make garchive' once you've started downloading required
# files (say with 'make paranoid-checksum'. Example:

GARCHIVEDIR = ../../garchive

# These are the standard directory name variables from all GNU
# makefiles.  They're also used by autoconf, and can be adapted
# for a variety of build systems.
# 
# TODO: set $(SYSCONFDIR) and $(LOCALSTATEDIR) to never use
# /usr/etc or /usr/var

##############################################################################
Here go the AGAI specific values:

DESTIMG ?= main
# as we want gar to use our "own" cmake, add /vol/ai/bin to the search path
PATH:=/vol/ai/bin:$(PATH)
export PATH

# a main prefix file is there to remember the value of main_prefix
## varibale during first invocation.
## In all successive run of gar this file is read and main_prefix is set
## to the value indicated in this fiel. This means: You only have to define 
## main_prefix for the first module you install from a gar location. Afterwards
## gar automatically chooses the same prefix again and again, until you set
## another, by e.g. export main_prefix="/tmp/somestrangeplaceforinstallation"
MAIN_PREFIX_FILE:=$(GARDIR)/gar_main_prefix

GAR_PROMPT_PREFIX="(read -p 'please set install prefix: ' userinput && echo $$(userinput))"
# Directory config for the "main" image
## if we have a configured prefix file in the root of the gar system
## (see above) use it, otherwise use $(HOME)/Desktop/gar-build
#main_prefix ?= $(shell cat $(MAIN_PREFIX_FILE) 2>/dev/null || echo $(HOME)/Desktop/gar-build)
main_prefix ?= $(shell cat $(MAIN_PREFIX_FILE) 2>/dev/null || (read -p 'please set install prefix: ' userinput && echo "$$userinput" && echo "$$userinput">$(MAIN_PREFIX_FILE)))

# Many modules can be build in parallel. This little trick determines 
## the number of CPUs (to be precise: number of cores). Modules can then 
## set MFLAGS=$(MFLAGS_NUMCPU) in order to enable best performance 
## compilation if supported by the respective make scheme
MFLAGS_NUMCPU=$(shell cat /proc/cpuinfo | grep processor | wc -l | sed "s/^\(..*\)/-j\1/")

# root for agai svn release (e.g. to trunk or tag)
## This is just for convinience, check in each module, where it get the
## source from!
#agai_svn_root=svnhttps://code.ai.techfak.uni-bielefeld.de/scm/ai/trunk
agai_svn_root=svnhttps://code.ai.techfak.uni-bielefeld.de/scm/ai

## This is just for convinience, check in each module, where it get the
## source from! This needs to be set to the patch of the release name
## e.g. branches/supercoolreleasename/ or tags/coolTag/
## and do not forget the / and the end...
agai_release_name=trunk/

# ****************************************************************************
# some absolute installation paths follow. These are *not* recommended, but
# for sake of simplicity and convinience, we allow *some* fixed installation
# of really bulky software packages...
# they might be used by *some* modules, but should be avoided in favor of
# self-contained releases (so put them to contrib/*)
#ESMERALA_prefix=/vol/esmeralda
RAVL_prefix=/vol/ravl/svnhead/RavlProject
MARY_prefix=/vol/mobirob/share/mary


##############################################################################
# additional AGAI specific rules:

# the rule to create the MAIN_PREFIX_FILE
remember_main_prefix:  $(MAIN_PREFIX_FILE)

# the rule to *really* create the MAIN_PREFIX_FILE
$(MAIN_PREFIX_FILE):
		@echo "*** remembering the prefix $(main_prefix) in file $@ ***"
		@echo "$(main_prefix)" > $@

install:	remember_main_prefix 

update:		clean build

##############################################################################

COOKIE_SUFFIX=$(shell pwd | sed "s@.*/\([^/]*\)/\([^/]*\)@\1/\2@")

#COOKIEROOTDIR=$(localstatedir)/gar/cookies/$(COOKIE_SUFFIX)
#COOKIEDIR=$(COOKIEROOTDIR)/$(COOKIE_SUFFIX)


main_exec_prefix = $(main_prefix)
main_bindir = $(main_exec_prefix)/bin
main_sbindir = $(main_exec_prefix)/sbin
main_libexecdir = $(main_exec_prefix)/libexec
main_datadir = $(main_prefix)/share
main_sysconfdir = $(main_prefix)/etc
main_sharedstatedir = $(main_prefix)/share
main_localstatedir = $(main_prefix)/var
main_libdir = $(main_exec_prefix)/lib
main_infodir = $(main_prefix)/info
main_lispdir = $(main_prefix)/share/emacs/site-lisp
main_includedir = $(main_prefix)/include
main_mandir = $(main_prefix)/man
main_docdir = $(main_prefix)/share/doc/$(DISTNAME)
main_sourcedir = $(main_prefix)/src
main_licensedir = $(main_prefix)/licenses

prefix = $($(DESTIMG)_prefix)
exec_prefix = $($(DESTIMG)_exec_prefix)
bindir = $($(DESTIMG)_bindir)
sbindir = $($(DESTIMG)_sbindir)
libexecdir = $($(DESTIMG)_libexecdir)
datadir = $($(DESTIMG)_datadir)
sysconfdir = $($(DESTIMG)_sysconfdir)
sharedstatedir = $($(DESTIMG)_sharedstatedir)
localstatedir = $($(DESTIMG)_localstatedir)
libdir = $($(DESTIMG)_libdir)
infodir = $($(DESTIMG)_infodir)
lispdir = $($(DESTIMG)_lispdir)
includedir = $($(DESTIMG)_includedir)
mandir = $($(DESTIMG)_mandir)
docdir = $($(DESTIMG)_docdir)
sourcedir = $($(DESTIMG)_sourcedir)
licensedir = $($(DESTIMG)_licensedir)

# the DESTDIR is used at INSTALL TIME ONLY to determine what the
# filesystem root should be.  Each different DESTIMG has its own
# DESTDIR.
main_DESTDIR ?=

DESTDIR = $(main_DESTDIR)

BUILD_PREFIX ?= $(prefix)

# allow us to link to libraries we installed
CPPFLAGS += -I$(DESTDIR)$(includedir)
CFLAGS += -I$(DESTDIR)$(includedir)
LDFLAGS += -L$(DESTDIR)$(libdir) -Wl,-rpath,$(libdir)

# allow us to use programs we just built
PATH := $(DESTDIR)$(bindir):$(DESTDIR)$(sbindir):$(DESTDIR)$(BUILD_PREFIX)/bin:$(DESTDIR)$(BUILD_PREFIX)/sbin:$(PATH)
LD_LIBRARY_PATH := $(DESTDIR)$(libdir):$(DESTDIR)$(BUILD_PREFIX)/lib:$(LD_LIBRARY_PATH)

# 
PKG_CONFIG_PATH:=$(DESTDIR)$(libdir)/pkgconfig:$(PKG_CONFIG_PATH):/usr/lib/pkgconfig
export PKG_CONFIG_PATH

# Sensible testing defaults
CFLAGS += -O2 -pipe

# Specific compiler settings
# GCC_BASE=/vol/gcc
# CC=/vol/gcc/bin/gcc
# CXX=/vol/gcc/bin/g++
# gcc-3 does require its lib but doesn't link with correct run-path by default
# LDFLAGS+=-L/vol/gcc/lib -Wl,-rpath,/vol/gcc/lib
# export path to specific gcc
# PATH := $(GCC_BASE)/bin:$(PATH)

# Old compiler
#CC = gcc-2.95
#CXX = gcc-2.95
# Intel Pentium Pro and above
#CFLAGS += -march=pentiumpro

# GCC 3.2
#CC = gcc-3.2
#CXX = g++-3.2
# GCC 3.3
#CC = gcc-3.3
#CXX = g++-3.3

# GCC 3.x CFLAGS
# Intel Pentium Pro and above
#CFLAGS += -march=pentiumpro
# My iBook
#CFLAGS += -march=750

# Intel's ICC
#CC = /opt/intel/compiler70/ia32/bin/icc
#CXX = /opt/intel/compiler70/ia32/bin/icc

# ccache, if you've got it - you must set CC and CXX above too!
#CC := ccache $(CC)
#CXX := ccache $(CXX)

# Equalise CFLAGS and CXXFLAGS
CXXFLAGS := $(CFLAGS)

# If you want to use a different version of python everywhere
# change this
PYTHON = $(shell which python)

# Let's see if we can get gtk-doc going 100%
XML_CATALOG_FILES += $(DESTDIR)$(sysconfdir)/xml/catalog

# GNOME CVS root if we need it
GNOMECVSROOT = ":pserver:jdub@cvs.gnome.org:/cvs/gnome"

# Put these variables in the environment during the
# configure build and install stages
STAGE_EXPORTS = DESTDIR prefix exec_prefix bindir sbindir libexecdir datadir includedir
STAGE_EXPORTS += sysconfdir sharedstatedir localstatedir libdir infodir lispdir
STAGE_EXPORTS += mandir docdir sourcedir PKG_CONFIG_PATH
STAGE_EXPORTS += CPPFLAGS CFLAGS CXXFLAGS LDFLAGS
STAGE_EXPORTS += CC CXX PYTHON

CONFIGURE_ENV += $(foreach TTT,$(STAGE_EXPORTS),$(TTT)="$($(TTT))")
BUILD_ENV += $(foreach TTT,$(STAGE_EXPORTS),$(TTT)="$($(TTT))")
INSTALL_ENV += $(foreach TTT,$(STAGE_EXPORTS),$(TTT)="$($(TTT))")
MANIFEST_ENV += $(foreach TTT,$(STAGE_EXPORTS),$(TTT)="$($(TTT))")

# Global environment
export PATH LD_LIBRARY_PATH #LD_PRELOAD
export PKG_CONFIG_PATH XML_CATALOG_FILES GNOMECVSROOT

# prepend the local file listing
FILE_SITES = file://$(FILEDIR)/ file://$(GARCHIVEDIR)/

# Extra libs to include with gar.mk
#EXTRA_LIBS = 
