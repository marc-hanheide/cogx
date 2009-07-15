"""
All global data is stored here (hopefully not too much).
"""
from os.path import abspath, join, dirname
import utils

src_path = abspath(dirname(__file__))  # where this file resides
main_path = dirname(src_path)          # one below this directory

CONFIG_FN = "config.ini"
config = utils.load_config_file(join(src_path, CONFIG_FN))
