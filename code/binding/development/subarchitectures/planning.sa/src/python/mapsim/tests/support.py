import os, sys
from os.path import dirname, abspath

# add MAPSIM main dir to the path and make sure you are in the test dir
TEST_DIR = dirname(abspath(__file__))
MAPSIM_DIR = dirname(TEST_DIR)  # one below
sys.path[1:1] = [MAPSIM_DIR]
