.DEFAULT:	alldeps

alldeps:
	(for f in */*/Makefile; do echo "installing in `dirname $$f`"; $(MAKE) -C `dirname $$f` install; done)

%:
	(for f in */*/Makefile; do echo "$@-ing in `dirname $$f`"; $(MAKE) -C `dirname $$f` $@; done)
	
