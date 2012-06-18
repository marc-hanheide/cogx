# Author: Marko Mahnič
# Created: 2012-06-15

.PHONY: default generated

default: generated

generated: dialogue_gen.inc

resources=dialogue.ui dialogue.js
dialogue_gen.inc: Makefile res2c.py $(resources)
	python res2c.py --c11 $@ $(resources)
