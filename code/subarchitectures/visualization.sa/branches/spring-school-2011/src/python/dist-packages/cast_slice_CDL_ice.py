#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

# slice2py is generating "import cast_slice_CDL_ice" instead of CDL_ice
# this file is a hack that imports the correct one.
from CDL_ice import *
