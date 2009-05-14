#! /usr/bin/env python
# -*- coding: latin-1 -*-

"""[describe program here]

usage: experiments.py [options] runs_file
  -d, --debug : show all log messages
"""

import os.path

import config
import utils

default_options = dict(debug=False)   # store default values for options here 

def parse_command_line():
    # parse options and required arguments from docstring automatically
    from myoptionparse import OptionParser
    parser = OptionParser(from_doc=__doc__)
    parser.set_defaults_from(default_options)
    options, args = parser.parse_args()
    options.optional_arguments = args
    globals()["options"] = options
    
def load_runs_file(runs_file):
    def gen(runs_file):
        domain = None
        suite = None
        settings = None
        if not os.path.exists(runs_file):
            runs_file = os.path.join(config.run_configs_dir, runs_file)
        else:
            config.run_configs_dir = os.path.dirname(os.path.abspath(runs_file))
        for i, line in enumerate(open(runs_file)):
            line = line.strip()
            if not line or line[0] == '#':
                continue
            tokens = line.split()
            first = tokens[0].lower()
            if first == "domain":
                domain = tokens[1]
                settings = None # for a new domain, new settings must be set
                suite = None  # for a new domain, a new suite must be set
            elif first == "suite":
                suite = tokens[1]
            elif first == "settings":
                settings = tokens[1]
            elif first == "stop":
                return
            else:
                assert domain, "No domain name set in %s, line %d" % (runs_file, i+1)
                assert settings, "No settings file given in %s, line %d" % (runs_file, i+1)
                assert suite, "No suite name set in %s, line %d" % (runs_file, i+1)
                yield (domain, suite, tokens[0], settings)
    return list(gen(runs_file))

def read_settings_gen(lines):
    current_setting = {}
    for line in lines:
        if not line:
            continue
        elif line.startswith("setting"):
            if current_setting:
                yield current_setting
            current_setting = {}
        key, val = line.split()
        current_setting[key] = utils.str2python_obj(val)
    if current_setting:
        yield current_setting

def load_settings_file(filename):
    filename = os.path.join(config.mapsim_dir, filename)
    lines = open(filename).read().split("\n")
    settings = list(read_settings_gen(lines))
    return settings



if __name__ == "__main__":
    parse_command_line()
    runs = list(load_runs_file(options.runs_file))
    print "\nRUNS loaded:"
    for run in runs:
        print run
