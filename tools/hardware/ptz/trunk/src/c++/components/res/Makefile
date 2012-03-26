
.PHONY: default generated

default: generated

generated: ptucontrol.inc

resources=ptucontroller.ui ptucontroller.js
ptucontrol.inc: Makefile res2c.py $(resources)
	python res2c.py --c03 $@ $(resources)
