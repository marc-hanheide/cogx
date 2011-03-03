"""
All global data is stored here (hopefully not too much).
"""
from os.path import abspath, join, dirname
import utils

src_path = abspath(dirname(__file__))  # where this file resides
main_path = dirname(src_path)          # one below this directory

def load_config_file(fn, cfg_path=src_path, **kwargs):
    """load a config file and all values defined there to the return value
    as fields, i.e., you can then access these values as
    config.somefield.somesubfield"""
    # The following dict sets some base values that can be referred to
    # in the config file via string interpolation
    base_config = dict(src_path=src_path, **kwargs)
    return utils.load_config_file(join(cfg_path, fn), base_dict=base_config)

CONFIG_FN = "config.ini"
CMAKE_CONFIG_FN = "config.auto"

config_autogen = load_config_file(CMAKE_CONFIG_FN)
config = load_config_file(CONFIG_FN, **config_autogen.__dict__)
config.__dict__.update(config_autogen.__dict__)
