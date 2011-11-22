.DEFAULT:	dora george

dora:
	sudo $(MAKE) -C cogx/dora-y4 install

george:
	sudo $(MAKE) -C cogx/U11.10-george-y4_450_x64 install
	



%:
	(for f in */*/Makefile; do echo "$@-ing in `dirname $$f`"; $(MAKE) -C `dirname $$f` $@; done)
	