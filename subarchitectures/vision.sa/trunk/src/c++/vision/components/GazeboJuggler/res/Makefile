
.PHONY: default generated

default: generated

generated: juggler_gen.inc

resources=juggler.ui juggler.js
juggler_gen.inc: Makefile res2c.py $(resources)
	python res2c.py --c11 $@ $(resources)
