#!/bin/sh


make -C BUILD clean; space; make -j 8 -C BUILD install  VERBOSE=1 2> errors ; grep error errors
