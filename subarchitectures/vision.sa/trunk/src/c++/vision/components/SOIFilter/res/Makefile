
.PHONY: default generated

default: generated

generated: ptuctrl.inc

ptuctrl.inc: ptucontroller.ui ptucontroller.js
	python res2c11.py
