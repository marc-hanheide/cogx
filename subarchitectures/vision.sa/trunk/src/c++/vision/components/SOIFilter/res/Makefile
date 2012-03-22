
.PHONY: default generated

default: generated

generated: ptuctrl.inc

resources=ptucontroller.ui ptucontroller.js
ptuctrl.inc: Makefile $(resources)
	python res2c11.py $@ $(resources)
