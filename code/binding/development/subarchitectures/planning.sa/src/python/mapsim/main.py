#! /usr/bin/env python
# -*- coding: latin-1 -*-


"""MAPSIM: a Multiagent Planning simulation environment.

usage: %prog [options] 
  -r, --reporter : report on simulation [default: %default]
  -a, --ack_mode=ACK : acknowledgments mode  [default: %default]
  -s, --sensing_mode=SENSING : sensing mode  [default: %default]
  -k, --key : wait for keypress after each executed action [default: %default]
  -d, --debug : show all log messages [default: %default]
  -v, --verbosity=VERBOSITY : general verbosity level [default: %default]
  -i, --info=INFO : info on specific parts of MAPSIM [default: %default]
  -e, --experiments=EXPERIMENTS: experiment suite [default: %default]
  -t, --test=TEST : test scenario [default: %default]
  -c, --create_functional_test=NAME : add results of this run as new functional test [default: %default]
  -f, --test_file=FILE : write the test into this filename [default: tests/files/NAME.rep]
  -p, --profile : profile this run [default: %default]
  -V, --version : show the MAPSIM version
  --speech : use text2speech engine for verbalizing dialogues [default: %default] 
  --mp3=filename : create an mp3 file of the interaction (only with --speech)
  --random=SEED : set random seed  [default: %default]
"""

"""
verbosity levels:
    0    : silent
    1    : reporter output
    2    : presentation output
    9999 : debug mode; all log messages printed

acknowledgment modes (can be combined bit-wise):
    1    : acknowledge request acceptance
    2    : acknowledge subgoal achievement has been noticed

sensing modes (can be combined bit-wise):
    1    :  report positive observations
    2    :  report negative observations

info modes:
    10   : infos on temporary subgoals (TSGs)
"""

version = "0.0.1"
version_info = """MAPSIM %(version)s""" % locals()

import sys, os, os.path
from os.path import dirname, abspath
import datetime

# make sure both the mapsim dir and the planning dir are in the path
MAPSIM_DIR = dirname(abspath(__file__))
PLANNING_DIR = os.path.join(MAPSIM_DIR, "planning")
for path in [MAPSIM_DIR, PLANNING_DIR]:
    if path not in sys.path:
        sys.path[1:1] = [path]


import config
from config import log
from constants import *
import simulation
import planning
import experiments
import tests
from tests import functional_tests
import utils

def parse_command_line():
    # parse options and required arguments from docstring automatically
    from myoptionparse import OptionParser
    parser = OptionParser(from_doc=__doc__)
    parser.set_defaults_from(config.__dict__)
    options, args = parser.parse_args()
    if options.version:
        print version_info
        sys.exit()
    if args:
        print "\nMAPSIM: Too many command-line arguments!  Call was:", " ".join(sys.argv)
        parser.print_help(sys.stderr)
        stop()
    if not (options.test or options.experiments):
        print "\nMAPSIM must be called with either -e or -t option.\n"
        parser.print_help(sys.stderr)
        stop()
    if options.mp3 and not options.speech:
        stop("MAPSIM: --mp3 option only usable with --speech.")
    if options.reporter:
        #options.verbosity = max(options.verbosity, REPORTER)
        config.reporter_verbalization = True
    if options.key:
        config.keypress_after_each_action = True
    if options.debug:
        options.verbosity = max(options.verbosity, DEBUG)
        options.verbosity_file = options.verbosity
        config.log2screen = True
    config.__dict__.update(options.__dict__)
    if config.experiments:
        config.test_suite = experiments.load_runs_file(options.experiments)
        config.runs_name = os.path.basename(config.experiments)
    elif config.test:
        setup_individual_run(config.test)
        config.runs_name = "single-runs"
        
def fake_command_line_args(options):
    if isinstance(options,basestring):
        options = options.split()
    sys.argv[1:] = options

def load_domain(dom_name):
    path = [config.domains_path, dom_name, "module"]
    dom_file = ".".join(path)
    try:
        domain_module = __import__(dom_file, globals(), locals(), [dom_name])
    except ImportError, e:
        sys.exit("Domain %s does not exist!" % dom_file)
    return domain_module

def setup_individual_run(scenario_path):
    dom_suite_scen_set = []
    p = scenario_path
    while p:
        p, base = os.path.split(p)
        if not base or base == "scenarios":
            continue
        if base == "domains":
            break
        dom_suite_scen_set.append(base)
    else:
        raise RuntimeError("Could not determine domain name for scenario %s." % scenario_path)
    dom_suite_scen_set.reverse()
    setting = config.dummy_setting
    # look for specific setting
    p = scenario_path
    while p:
        setting_path = os.path.join(p, config.STD_SETTING_FN)
        if os.path.exists(setting_path):
            setting = setting_path
            break
        p, base = os.path.split(p)
        if base == "domains":
            break
    dom_suite_scen_set.append(setting)
    config.test_suite = [dom_suite_scen_set]

def make_unique_dir(adir):
    num = 0
    while True:
        p = os.path.join(adir, "num%s" % str(num).zfill(3))
        if os.path.exists(p):
            num += 1
        else:
            os.makedirs(p)
            return p    

def setup_run(runs_name):
    time_str = datetime.datetime.now().strftime("%Y-%m-%d.%H:%M:%S")
    dirs = (config.common_data_path, runs_name, time_str)
    current_run_path = os.path.join(*dirs)
    current_run_path = make_unique_dir(current_run_path)
    config.link(current_run_path, config.current_run_dir)

def setup_data_directory(runs_name, domain, suite, prob):
    # find scenario
    path = [config.domains_path, domain, "scenarios", suite, prob]
    config.scenario_path = os.path.join(*path)
    if not os.path.exists(config.scenario_path):
        print "Scenario file or directory %s does not exist!" % config.scenario_path
        print "current dir", os.getcwd()
        return False
    # setup data directory for this scenario
    dirs = [config.current_run_dir, domain, suite]
    data_dir = os.path.join(*dirs)
    return create_and_link_to_data_dir(data_dir)

def create_and_link_to_data_dir(data_dir):
    data_dir = make_unique_dir(data_dir)
    log("Creating data directory for '%s' scenario: %s" % (config.runs_name, data_dir))
    config.link(data_dir, config.current_data_dir)
    return True

def stop(msg=None, code=1):
    if msg:
        print msg
    sys.exit(code)

def eval_stats():
    stats = config.statistics
    allstats = config.all_statistics
    stats["simulator"] = stats[config.simulation]
    allstats[config.scenario] = stats
    solved = len([stat for stat in allstats.values() if stat["simulator"].solved])
    log("MAIN LOOP: %d solved, %d unsolved yet." % (solved, len(allstats)-solved))
    config.statistics = {}

def final_stats():
    log("\nOverall statistics for test suite '%s':" % (config.runs_name))
    allstats = config.all_statistics
    for scenario_name in allstats:
        stat = allstats[scenario_name]
        solved = stat["simulator"].solved
        log("%s was %s." % (scenario_name, ("solved" if solved else "unsolved")))

def cleanup_after_run():
    """ finish simulation gracefully even if it crashed for some reason """
    config.report_file.close()
    if config.speech:
        import speak
        speak.after_run()
    
def main_loop():
    if config.random:
        import random
        random.seed(config.random)
    current_domain = None
    setup_run(config.runs_name)
    for (domain_name, suite_name, prob_name, settings_name) in config.test_suite:
        config.suite_name = suite_name
        config.prob_name = prob_name
        if domain_name != current_domain:
            # switch to a new domain
            current_domain = config.domain_name = domain_name
            config.domain_module = load_domain(config.domain_name)
            settings = experiments.load_settings_file(settings_name)
        for setting in settings:
            dirs_done = setup_data_directory(config.runs_name, current_domain, suite_name, prob_name)
            if not dirs_done:
                continue
            sim = config.domain_module.Simulation()
            sim.setup(setting, prob_name)
            try:
                sim.run()
            except KeyboardInterrupt:
                print "MAPSIM terminates early (keyboard interruption)."
                sys.exit()
            finally:
                cleanup_after_run()
            eval_stats()
    final_stats()

def testmain():
    fake_command_line_args(" ")
    parse_command_line()

    if config.profile:
        import cProfile
        profile_stats = ".profiling.stats"
        cProfile.run('main_loop()', profile_stats)
        import tools.profiling
        tools.profiling.analyze_stats(profile_stats)
    else:
        main_loop()
        
def main(cmd_line_args=None):
    config.load_configuration(config.config_filename)
    if cmd_line_args:
        fake_command_line_args(cmd_line_args)
    parse_command_line()
        
    if config.profile:
        import cProfile
        profile_stats = ".profiling.stats"
        cProfile.run('main_loop()', profile_stats)
        import tools.profiling
        tools.profiling.analyze_stats(profile_stats)
    else:
        main_loop()
    
    if config.create_functional_test:
        tests.functional_tests.add_run_to_functional_tests(sys.argv)
