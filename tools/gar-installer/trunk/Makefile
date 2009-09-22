SUBDIRS=	contrib/* \
		cogx/* 



default:	
	@echo "available top-level targets:"
	@echo "  build, install, %-build, %-install, projperm, clean, mrproper"



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

projperm:
	$(MAKE) -C util/projperm install

clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done

mrproper:	clean
	rm -f gar_main_prefix
