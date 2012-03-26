
.PHONY: default generated

default: generated

generated: ptuctrl.inc

resources=ptucontroller.ui ptucontroller.js
ptuctrl.inc: Makefile res2c.py $(resources)
	python res2c.py --c11 $@ $(resources)
