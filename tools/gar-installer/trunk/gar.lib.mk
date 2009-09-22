#-*- mode: Fundamental; tab-width: 4; -*-
# ex:ts=4 sw=4
# $Id: gar.lib.mk,v 1.1 2006/01/30 17:32:58 swrede Exp $

# Copyright (C) 2001 Nick Moffitt
# 
# Redistribution and/or use, with or without modification, is
# permitted.  This software is without warranty of any kind.  The
# author(s) shall not be liable in the event that use of the
# software causes damage.

# cookies go here, so we have to be able to find them for
# dependency checking.
VPATH += $(COOKIEDIR)

# So these targets are all loaded into bbc.port.mk at the end,
# and provide actions that would be written often, such as
# running configure, automake, makemaker, etc.  
#
# The do- targets depend on these, and they can be overridden by
# a port maintainer, since they'e pattern-based.  Thus:
#
# extract-foo.tar.gz:
#	(special stuff to unpack non-standard tarball, such as one
#	accidentally named .gz when it's really bzip2 or something)
#
# and this will override the extract-%.tar.gz rule.

# convenience variable to make the cookie.
MAKECOOKIE = mkdir -p $(COOKIEDIR)/$(@D) &&  date >> $(COOKIEDIR)/$@
#################### FETCH RULES ####################

URLS = $(subst ://,//,$(foreach SITE,$(FILE_SITES) $(MASTER_SITES),$(addprefix $(SITE),$(DISTFILES))) $(foreach SITE,$(FILE_SITES) $(PATCH_SITES) $(MASTER_SITES),$(addprefix $(SITE),$(PATCHFILES))))


# Download the file if and only if it doesn't have a preexisting
# checksum file.  Loop through available URLs and stop when you
# get one that doesn't return an error code.
$(DOWNLOADDIR)/%:  
	@if test -f $(COOKIEDIR)/checksum-$*; then : ; else \
		echo " ==> Grabbing $@"; \
		for i in $(filter %/$*,$(URLS)); do  \
			echo " 	==> Trying $$i"; \
			$(MAKE) -s $$i || continue; \
			mv $(PARTIALDIR)/$* $@; \
			break; \
		done; \
		if test -r $@ ; then : ; else \
			echo '*** GAR GAR GAR!  Failed to download $@!  GAR GAR GAR! ***' 1>&2; \
			false; \
		fi; \
	fi

# download an http URL (colons omitted)
http//%: 
	@wget -T 10 -c -P $(PARTIALDIR) http://$*

# download an ftp URL (colons omitted)
ftp//%: 
	@wget -T 10 -c --passive-ftp -P $(PARTIALDIR) ftp://$*

# link to a local copy of the file
# (absolute path)
file///%: 
	@if test -f /$*; then \
		ln -sf /$* $(PARTIALDIR)/$(notdir $*); \
	else \
		false; \
	fi

# link to a local copy of the file
# (relative path)
file//%: 
	@if test -f $*; then \
		ln -sf "$(CURDIR)/$*" $(PARTIALDIR)/$(notdir $*); \
	else \
		false; \
	fi

# Using Jeff Waugh's rsync rule.
# DOES NOT PRESERVE SYMLINKS!
rsync//%: 
	@rsync -azvLP rsync://$* $(PARTIALDIR)/

# Using Jeff Waugh's scp rule
scp//%:
	@scp -C $* $(PARTIALDIR)/

#################### LOG RULES ####################

log-%:
	@mkdir -p log
	@$(MAKE) $* 2>&1 | tee $(LOGDIR)/$*	

#################### CHECKSUM RULES ####################

# check a given file's checksum against $(CHECKSUM_FILE) and
# error out if it mentions the file without an "OK".
checksum-%: $(CHECKSUM_FILE) 
	@echo " ==> Running checksum on $*"
	@if grep -- '$*' $(CHECKSUM_FILE); then \
		if LC_ALL="C" LANG="C" md5sum -c $(CHECKSUM_FILE) 2>&1 | grep -- '$*' | grep -v ':[ ]\+OK'; then \
			echo '*** GAR GAR GAR!  $* failed checksum test!  GAR GAR GAR! ***' 1>&2; \
			false; \
		else \
			echo 'file $* passes checksum test!'; \
			$(MAKECOOKIE); \
		fi \
	else \
		echo '*** GAR GAR GAR!  $* not in $(CHECKSUM_FILE) file!  GAR GAR GAR! ***' 1>&2; \
		false; \
	fi
		

#################### GARCHIVE RULES ####################

# while we're here, let's just handle how to back up our
# checksummed files

$(GARCHIVEDIR)/%: $(GARCHIVEDIR)
	@if [ -L $(DOWNLOADDIR)/$* ]; then :; else \
		cp -Lr $(DOWNLOADDIR)/$* $@; \
	fi


#################### EXTRACT RULES ####################

# rule to extract uncompressed tarballs
tar-extract-%:
	@echo " ==> Extracting $(DOWNLOADDIR)/$*"
	@tar -xf $(DOWNLOADDIR)/$* -C $(EXTRACTDIR)
	@$(MAKECOOKIE)

# rule to extract files with tar xzf
tar-gz-extract-%:
	@echo " ==> Extracting $(DOWNLOADDIR)/$*"
	@gzip -dc $(DOWNLOADDIR)/$* | tar -xf - -C $(EXTRACTDIR)
	@$(MAKECOOKIE)

# rule to extract files with tar and bzip
tar-bz-extract-%:
	@echo " ==> Extracting $(DOWNLOADDIR)/$*"
	@bzip2 -dc $(DOWNLOADDIR)/$* | tar -xf - -C $(EXTRACTDIR)
	@$(MAKECOOKIE)

# rule to extract files with unzip
zip-extract-%:
	@echo " ==> Extracting $(DOWNLOADDIR)/$*"
	@unzip $(DOWNLOADDIR)/$* -d $(EXTRACTDIR)
	@$(MAKECOOKIE)

# this is a null extract rule for files which are constant and
# unchanged (not archives)
cp-extract-%:
	@echo " ==> Copying $(DOWNLOADDIR)/$*"
	@cp $(DOWNLOADDIR)/$* $(WORKDIR)/
	@$(MAKECOOKIE)

#gets the meat of a .deb into $(WORKSRC)
deb-bin-extract-%:
	@echo " ==> Extracting $(DOWNLOADDIR)/$*"
	@ar x $(DOWNLOADDIR)/$*
	@rm debian-binary && mv *.tar.gz $(DOWNLOADDIR) && mkdir $(WORKSRC) && tar -xvz -C $(WORKSRC) -f $(DOWNLOADDIR)/data.tar.gz
	@$(MAKECOOKIE)

### EXTRACT FILE TYPE MAPPINGS ###
# These rules specify which of the above extract action rules to use for a
# given file extension.  Often support for a given extract type can be handled
# by simply adding a rule here.

extract-%.tar: tar-extract-%.tar
	@$(MAKECOOKIE)

extract-%.tar.gz: tar-gz-extract-%.tar.gz
	@$(MAKECOOKIE)

extract-%.tar.Z: tar-gz-extract-%.tar.Z
	@$(MAKECOOKIE)

extract-%.tgz: tar-gz-extract-%.tgz
	@$(MAKECOOKIE)

extract-%.taz: tar-gz-extract-%.taz
	@$(MAKECOOKIE)

extract-%.tar.bz: tar-bz-extract-%.tar.bz
	@$(MAKECOOKIE)

extract-%.tar.bz2: tar-bz-extract-%.tar.bz2
	@$(MAKECOOKIE)

extract-%.tbz: tar-bz-extract-%.tbz
	@$(MAKECOOKIE)

extract-%.zip: zip-extract-%.zip
	@$(MAKECOOKIE)

extract-%.ZIP: zip-extract-%.ZIP
	@$(MAKECOOKIE)

extract-%.deb: deb-bin-extract-%.deb
	@$(MAKECOOKIE)


# anything we don't know about, we just assume is already
# uncompressed and unarchived in plain format
extract-%: cp-extract-%
	@$(MAKECOOKIE)


#################### PATCH RULES ####################

PATCHDIR ?= $(WORKSRC)
PATCHDIRLEVEL ?= 1
PATCHDIRFUZZ ?= 2
GARPATCH = patch -d$(PATCHDIR) -p$(PATCHDIRLEVEL) -F$(PATCHDIRFUZZ)
BASEWORKSRC = $(shell basename $(WORKSRC))

# apply bzipped patches
bz-patch-%:
	@echo " ==> Applying patch $(DOWNLOADDIR)/$*"
	@bzip2 -dc $(DOWNLOADDIR)/$* |$(GARPATCH)
	@$(MAKECOOKIE)

# apply gzipped patches
gz-patch-%:
	@echo " ==> Applying patch $(DOWNLOADDIR)/$*"
	@gzip -dc $(DOWNLOADDIR)/$* |$(GARPATCH)
	@$(MAKECOOKIE)

# apply normal patches
normal-patch-%:
	@echo " ==> Applying patch $(DOWNLOADDIR)/$*"
	@$(GARPATCH) <$(DOWNLOADDIR)/$*
	@$(MAKECOOKIE)

# This is used by makepatch
%/gar-base.diff:
	@echo " ==> Creating patch $@"
	@EXTRACTDIR=$(SCRATCHDIR) COOKIEDIR=$(SCRATCHDIR)-$(COOKIEDIR) $(MAKE) extract
	@mv $(SCRATCHDIR)/$(BASEWORKSRC) $(WORKSRC).orig
	@cd $(WORKDIR); \
		if diff --speed-large-files --minimal -ru $(DIFFARGS) $(BASEWORKSRC).orig $(BASEWORKSRC) > gar-base.diff; then :; else \
			cd -; \
			mv -f $(WORKDIR)/gar-base.diff $@; \
		fi

### PATCH FILE TYPE MAPPINGS ###
# These rules specify which of the above patch action rules to use for a given
# file extension.  Often support for a given patch format can be handled by
# simply adding a rule here.

patch-%.bz: bz-patch-%.bz
	@$(MAKECOOKIE)

patch-%.bz2: bz-patch-%.bz2
	@$(MAKECOOKIE)

patch-%.gz: gz-patch-%.gz
	@$(MAKECOOKIE)

patch-%.Z: gz-patch-%.Z
	@$(MAKECOOKIE)

patch-%.diff: normal-patch-%.diff
	@$(MAKECOOKIE)

patch-%.patch: normal-patch-%.patch
	@$(MAKECOOKIE)

#################### CONFIGURE RULES ####################

TMP_DIRPATHS = --prefix=$(prefix) --exec_prefix=$(exec_prefix) --bindir=$(bindir) --sbindir=$(sbindir) --libexecdir=$(libexecdir) --datadir=$(datadir) --sysconfdir=$(sysconfdir) --sharedstatedir=$(sharedstatedir) --localstatedir=$(localstatedir) --libdir=$(libdir) --infodir=$(infodir) --lispdir=$(lispdir) --includedir=$(includedir) --mandir=$(mandir)

NODIRPATHS += --lispdir

DIRPATHS = $(filter-out $(addsuffix %,$(NODIRPATHS)), $(TMP_DIRPATHS))

# configure a package that has an autoconf-style configure
# script.
configure-%/configure:
	@echo " ==> Running configure in $*"
	@cd $* && $(CONFIGURE_ENV) ./configure $(CONFIGURE_ARGS)
	@$(MAKECOOKIE)

configure-%/autogen.sh:
	@echo " ==> Running autogen.sh in $*"
	@cd $* && $(CONFIGURE_ENV) ./autogen.sh $(CONFIGURE_ARGS)
	@$(MAKECOOKIE)

# configure a package that uses imake
# FIXME: untested and likely not the right way to handle the
# arguments
configure-%/Imakefile: 
	@echo " ==> Running xmkmf in $*"
	@cd $* && $(CONFIGURE_ENV) xmkmf $(CONFIGURE_ARGS)
	@$(MAKECOOKIE)

#################### BUILD RULES ####################

# build from a standard gnu-style makefile's default rule.
build-%/Makefile:
	@echo " ==> Running make in $*"
	@$(BUILD_ENV) $(MAKE) $(PARALLELMFLAGS) $(foreach TTT,$(BUILD_OVERRIDE_DIRS),$(TTT)="$($(TTT))") -C $* $(BUILD_ARGS)
	@$(MAKECOOKIE)

build-%/makefile:
	@echo " ==> Running make in $*"
	@$(BUILD_ENV) $(MAKE) $(PARALLELMFLAGS) $(foreach TTT,$(BUILD_OVERRIDE_DIRS),$(TTT)="$($(TTT))") -C $* $(BUILD_ARGS)
	@$(MAKECOOKIE)

build-%/GNUmakefile:
	@echo " ==> Running make in $*"
	@$(BUILD_ENV) $(MAKE) $(PARALLELMFLAGS) $(foreach TTT,$(BUILD_OVERRIDE_DIRS),$(TTT)="$($(TTT))") -C $* $(BUILD_ARGS)
	@$(MAKECOOKIE)


#################### STRIP RULES ####################
# The strip rule should probably strip uninstalled binaries.
# TODO: Seth, what was the exact parameter set to strip that you
# used to gain maximal space on the LNX-BBC?

# Strip all binaries listed in the manifest file
# TODO: actually write it!
#  This will likely become almost as hairy as the actual
#  installation code.
strip-$(MANIFEST_FILE):
	@echo "Not finished"

# The Makefile must have a "make strip" rule for this to work.
strip-%/Makefile:
	@echo " ==> Running make strip in $*"
	@$(BUILD_ENV) $(MAKE) -C $* $(BUILD_ARGS) strip
	@$(MAKECOOKIE)

#################### INSTALL RULES ####################

# just run make install and hope for the best.
install-%/Makefile:
	@echo " ==> Running make install in $*"
	@$(INSTALL_ENV) $(MAKE) DESTDIR=$(DESTDIR) $(foreach TTT,$(INSTALL_OVERRIDE_DIRS),$(TTT)="$(DESTDIR)$($(TTT))") -C $* $(INSTALL_ARGS) install
	@$(MAKECOOKIE)

install-%/makefile:
	@echo " ==> Running make install in $*"
	@$(INSTALL_ENV) $(MAKE) DESTDIR=$(DESTDIR) $(foreach TTT,$(INSTALL_OVERRIDE_DIRS),$(TTT)="$(DESTDIR)$($(TTT))") -C $* $(INSTALL_ARGS) install
	@$(MAKECOOKIE)

install-%/GNUmakefile:
	@echo " ==> Running make install in $*"
	@$(INSTALL_ENV) $(MAKE) DESTDIR=$(DESTDIR) $(foreach TTT,$(INSTALL_OVERRIDE_DIRS),$(TTT)="$(DESTDIR)$($(TTT))") -C $* $(INSTALL_ARGS) install
	@$(MAKECOOKIE)

# LICENSE INSTALLATION

LICENSEDIR = $(GARDIR)/licenses

GPL_LICENSE_TEXT = $(LICENSEDIR)/GPL.txt
GPL1_LICENSE_TEXT = $(LICENSEDIR)/GPL1.txt
GPL2_LICENSE_TEXT = $(LICENSEDIR)/GPL2.txt
LGPL_LICENSE_TEXT = $(LICENSEDIR)/LGPL.txt
LGPL2_LICENSE_TEXT = $(LICENSEDIR)/LGPL2.txt
LGPL2_1_LICENSE_TEXT = $(LICENSEDIR)/LGPL2_1.txt
BSD_LICENSE_TEXT = $(LICENSEDIR)/BSD.txt
IBMPL_LICENSE_TEXT = $(LICENSEDIR)/IBMPL.txt
ISC_LICENSE_TEXT = $(LICENSEDIR)/ISC.txt
MIT_LICENSE_TEXT = $(LICENSEDIR)/MIT.txt
MIT_X_LICENSE_TEXT = $(LICENSEDIR)/MIT.txt
MPL_LICENSE_TEXT = $(LICENSEDIR)/MPL.txt
Artistic_LICENSE_TEXT = $(LICENSEDIR)/Artistic.txt
Clarified_Artistic_LICENSE_TEXT = $(LICENSEDIR)/Clarified-Artistic.txt
Public_Domain_LICENSE_TEXT = $(LICENSEDIR)/public-domain.txt

install-license-%: $($*_LICENSE_TEXT)
	echo " ==> Installing $* license text"
	-install -d $(DESTDIR)$(licensedir)/$(GARNAME)/
	install -m 644 $($*_LICENSE_TEXT) $(DESTDIR)$(licensedir)/$(GARNAME)/
	$(MAKECOOKIE)

# pkg-config scripts

install-%-config:
	mkdir -p $(STAGINGDIR)/$(GARNAME)
	cp -f $(DESTDIR)$(bindir)/$*-config $(STAGINGDIR)/$(GARNAME)/
	$(MAKECOOKIE)

######################################
# Use a manifest file of the format:
# src:dest[:mode[:owner[:group]]]
#   as in...
# ${WORKSRC}/nwall:${bindir}/nwall:2755:root:tty
# ${WORKSRC}/src/foo:${sharedstatedir}/foo
# ${WORKSRC}/yoink:${sysconfdir}/yoink:0600

# Okay, so for the benefit of future generations, this is how it
# works:
# 
# First of all, we have this file with colon-separated lines.
# The $(shell cat foo) routine turns it into a space-separated
# list of words.  The foreach iterates over this list, putting a
# colon-separated record in $(ZORCH) on each pass through.
# 
# Next, we have the macro $(MANIFEST_LINE), which splits a record
# into a space-separated list, and $(MANIFEST_SIZE), which
# determines how many elements are in such a list.  These are
# purely for convenience, and could be inserted inline if need
# be.
MANIFEST_LINE = $(subst :, ,$(ZORCH)) 
MANIFEST_SIZE = $(words $(MANIFEST_LINE))

# So the install command takes a variable number of parameters,
# and our records have from two to five elements.  Gmake can't do
# any sort of arithmetic, so we can't do any really intelligent
# indexing into the list of parameters.
# 
# Since the last three elements of the $(MANIFEST_LINE) are what
# we're interested in, we make a parallel list with the parameter
# switch text (note the dummy elements at the beginning):
MANIFEST_FLAGS = notused notused --mode= --owner= --group=

# The following environment variables are set before the
# installation boogaloo begins.  This ensures that WORKSRC is
# available to the manifest and that all of the location
# variables are suitable for *installation* (that is, using
# DESTDIR)

MANIFEST_ENV += WORKSRC=$(WORKSRC)
# This was part of the "implicit DESTDIR" regime.  However:
# http://gar.lnx-bbc.org/wiki/ImplicitDestdirConsideredHarmful
#MANIFEST_ENV += $(foreach TTT,prefix exec_prefix bindir sbindir libexecdir datadir sysconfdir sharedstatedir localstatedir libdir infodir lispdir includedir mandir,$(TTT)=$(DESTDIR)$($(TTT)))

# ...and then we join a slice of it with the corresponding slice
# of the $(MANIFEST_LINE), starting at 3 and going to
# $(MANIFEST_SIZE).  That's where all the real magic happens,
# right there!
# 
# following that, we just splat elements one and two of
# $(MANIFEST_LINE) on the end, since they're the ones that are
# always there.  Slap a semicolon on the end, and you've got a
# completed iteration through the foreach!  Beaujolais!

# FIXME: using -D may not be the right thing to do!
install-$(MANIFEST_FILE):
	@echo " ==> Installing from $(MANIFEST_FILE)"
	$(MANIFEST_ENV) ; $(foreach ZORCH,$(shell cat $(MANIFEST_FILE)), install -Dc $(join $(wordlist 3,$(MANIFEST_SIZE),$(MANIFEST_FLAGS)),$(wordlist 3,$(MANIFEST_SIZE),$(MANIFEST_LINE))) $(word 1,$(MANIFEST_LINE)) $(word 2,$(MANIFEST_LINE)) ;)
	@$(MAKECOOKIE)

# stanard steps for installing a minit script
install-%.init:
	@echo " ==> Installing minit script $*"
	@install -D -m 755 $(WORKDIR)/$*.init $(DESTDIR)$(sysconfdir)/init.d/$*
	@if ! grep "^NOSTOP[ ]=" $(WORKDIR)/$*.init > /dev/null; then \
		install -d $(DESTDIR)$(sysconfdir)/rchalt.d $(DESTDIR)$(sysconfdir)/rcreboot.d ;\
		ln -sf ../init.d/$* $(DESTDIR)$(sysconfdir)/rchalt.d/K$* ;\
		ln -sf ../init.d/$* $(DESTDIR)$(sysconfdir)/rcreboot.d/K$* ;\
	fi
	@$(MAKECOOKIE)

#################### DEPENDENCY RULES ####################

# These two lines are here to grandfather in all the packages that use
# BUILDDEPS
IMGDEPS += build
build_DEPENDS = $(BUILDDEPS)

# Standard deps install into the standard install dir.  For the
# BBC, we set the includedir to the build tree and the libdir to
# the install tree.  Most dependencies work this way.

$(GARDIR)/%/$(COOKIEDIR)/install:
	@echo ' ==> Building $* as a dependency'
	@$(MAKE) -C $(GARDIR)/$* install DESTIMG=$(DESTIMG)

# builddeps need to have everything put in the build DESTIMG
#$(GARDIR)/%/$(COOKIEROOTDIR)/build.d/install:
#	@echo ' ==> Building $* as a build dependency'
#	@$(MAKE) -C $(GARDIR)/$* install	DESTIMG=build

# Source Deps grab the source code for another package
# XXX: nobody uses this, but it should really be more like
# $(GARDIR)/%/cookies/patch:
srcdep-$(GARDIR)/%:
	@echo ' ==> Grabbing source for $* as a dependency'
	@$(MAKE) -C $(GARDIR)/$* patch-p extract-p > /dev/null 2>&1 || \
	 $(MAKE) -C $(GARDIR)/$* patch

# Image deps create dependencies on package installations in
# images other than the current package's DESTIMG.
IMGDEP_TARGETS = $(foreach TTT,$($*_DEPENDS),$(subst xyzzy,$(TTT),$(GARDIR)/xyzzy/$(COOKIEROOTDIR)/$*.d/install))
imgdep-%:
	@test -z "$(strip $(IMGDEP_TARGETS))" || $(MAKE) DESTIMG="$*" $(IMGDEP_TARGETS)

# Igor's info and man gzipper rule
gzip-info-man: gzip-info gzip-man

gzip-info:
	find $(DESTDIR) -type f -iname *.info* -not -iname *.gz | \
        xargs -r gzip --force

gzip-man:
	find $(DESTDIR) -type f -iname *.[1-8] -size +2 -print | \
        xargs -r gzip --force

#include $(addprefix $(GARDIR)/,$(EXTRA_LIBS))

# Mmm, yesssss.  cookies my preciousssss!  Mmm, yes downloads it
# is!  We mustn't have nasty little gmakeses deleting our
# precious cookieses now must we?
.PRECIOUS: $(DOWNLOADDIR)/% $(COOKIEDIR)/% $(FILEDIR)/%
