SUBDIRS=	contrib/* \
		cogx/* 

ubuntu-packages:
	@echo "install all ubuntu packages"
	find -mindepth 3 -maxdepth 3 -name "Makefile" | xargs grep -l DEB_PACKAGE | xargs -i -- bash -c "make -C \`dirname {}\` ubuntu-install"

print-debs:
	@find -mindepth 3 -maxdepth 3 -name "Makefile" | xargs grep -l DEB_PACKAGE | xargs -i -- bash -c "$(MAKE) -s -C \`dirname {}\` print-debs"
	@echo

default:	
	@echo "available top-level targets:"
	@echo "  build, install, %-build, %-install, clean, mrproper, ubuntu-packages"



%-built:
	for dir in $*/*; do $(MAKE) -C $$dir; done

%-install:
	for dir in $*/*; do $(MAKE) -C $$dir install; done
%-clean:
	for dir in $*/*; do $(MAKE) -C $$dir clean; done
%-garchive:
	for dir in $*/*; do $(MAKE) -C $$dir garchive; done





build:	
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir; done

install:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir install; done

clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done

mrproper:	clean
	rm -f gar_main_prefix
