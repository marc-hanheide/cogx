
.PHONY: default generated

default: generated

generated: test_calc.inc

resources=calculator.ui calculator.js chat.ui
test_calc.inc: Makefile $(resources)
	python res2c11.py $@ $(resources)
