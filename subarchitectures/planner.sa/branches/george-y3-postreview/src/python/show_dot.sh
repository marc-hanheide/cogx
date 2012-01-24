#! /usr/bin/env bash

# Adopt the following commands according to 
# - your OS
# - graphviz installation
# - preferred output format
# - preferred viewer

dot $1 -O -Tpdf
xdg-open $1.pdf