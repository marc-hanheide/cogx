#------------------------------------------------------------------------------#

.PHONY: main
main: listener-monitor server control

.PHONY: all
all: listener-monitor server testrecord

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: listener-monitor
listener-monitor: utils/libtty.a
	make -C $@

.PHONY: server
server: audiosource utils/libtty.a
	make -C $@

.PHONY: control
control:
	make -C $@

.PHONY: testrecord
testrecord: audiosource utils/libtty.a
	make -C $@

.PHONY: audiosource
audiosource:
	make -C $@

utils/lib%.a:
	make -C utils lib$*.a

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	make -C utils clean
	make -C audiosource clean
	make -C listener-monitor clean
	make -C server clean
	make -C control clean
	make -C testrecord clean
