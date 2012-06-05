.DEFAULT:	dora george

dora:
	$(MAKE) -C cogx/dora-y4 install

george:
	$(MAKE) -C cogx/U11.10-george-y4_450_x64 install

v4r:
	$(MAKE) -C cogx/v4r install


gazebo:
	$(MAKE) -C contrib/gazebo install

cosycure:
	$(MAKE) -C cogx/cosycure install



%:
	(for f in */*/Makefile; do echo "$@-ing in `dirname $$f`"; $(MAKE) -C `dirname $$f` $@; done)
	
