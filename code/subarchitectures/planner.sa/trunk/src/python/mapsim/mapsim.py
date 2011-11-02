#! /usr/bin/env python
# -*- coding: latin-1 -*-

"""MAPSIM: a Multiagent Planning simulation environment.

usage: %prog [options] domain scenario
  -d, --debug : print debug messages to console and logfiles [default: %default]
  -v, --verbosity=VERBOSITY : general verbosity level for console output [default: %default]
  -l, --loglevel=VERBOSITY : general verbosity level for logging output [default: %default]
  -r, --random=SEED : random seed for the world state [default: %default]
  -n, --runs=ITERATIONS : number of different variants of the scenario [default: 1]
  -L, --learning_mode=(cluster|learn|test) : mode for the learning agent [default: %default]
  -m, --macro_filename=FILE : set the basename of the macro file. Defaults to domain name.
  -M, --macro_version=INDEX : read/write macros from this version of the macro file [default: %default]
  -e, --enable-macros=MACROS : enable these macros in eevry case [default: %default]
  -c, --config=FILES : load additional config files which overrides the builtin config [default: %default]
  -o, --log_out=FILE : Write logging output to this file [default: %default]
"""

from os.path import abspath, dirname
import random, logging
import cProfile

from standalone import pddl, config
import standalone.globals as global_vars

import simulation, agent

log = config.logger()

mapsim_path = abspath(dirname(__file__))  # where this file resides
CONFIG_FN = "config.ini"
LOG_CONFIG_FN = "logging.conf"

    
def parse_command_line():
    # parse options and required arguments from docstring automatically
    from myoptionparse import OptionParser
    parser = OptionParser(from_doc=__doc__)
    parser.set_defaults_from(global_vars.mapsim_config.__dict__)
    options, args = parser.parse_args()
    global_vars.mapsim_config.__dict__.update(options.__dict__)
    if options.config:
        for cfg in options.config.split(","):
            if not cfg.strip():
                continue
            add_config = global_vars.load_config_file(cfg.strip(), cfg_path="", **global_vars.config_autogen.__dict__)
            if 'standalone' in add_config.__dict__:
                global_vars.config.merge(add_config.standalone)
            if 'mapsim' in add_config.__dict__:
                global_vars.mapsim_config.merge(add_config.mapsim)
    if options.log_out:
        config.set_logfile(options.log_out)

if __name__ == '__main__':
    global_vars.mapsim_config = global_vars.load_config_file(CONFIG_FN, cfg_path=mapsim_path, **global_vars.config_autogen.__dict__)
    global_vars.logging_config = global_vars.load_config_file(LOG_CONFIG_FN, cfg_path=mapsim_path, **global_vars.config_autogen.__dict__)
    global_vars.update_config(CONFIG_FN, cfg_path=mapsim_path)

    config.logging_settings_from_dict(global_vars.logging_config.__dict__)

    #set default verbosity and loglevel from logging.conf
    global_vars.mapsim_config.__dict__['verbosity'] = 5-(logging.getLogger().level/10)
    global_vars.mapsim_config.__dict__['loglevel'] = 5-(logging.getLogger("file").level/10)
    
    parse_command_line()
    
    if global_vars.mapsim_config.verbosity is not None:
        level = logging.CRITICAL - (global_vars.mapsim_config.verbosity * 10)
        logging.getLogger().setLevel(level)

    if global_vars.mapsim_config.loglevel is not None:
        level = logging.CRITICAL - (global_vars.mapsim_config.loglevel * 10)
        logging.getLogger("file").setLevel(level)
    else:
        logging.getLogger("file").setLevel(logging.INFO)
        
    if global_vars.mapsim_config.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logging.getLogger("file").setLevel(logging.DEBUG)

    if global_vars.mapsim_config.random is None:
        global_vars.mapsim_config.random = hash(random.random())

    domain = pddl.load_domain(global_vars.mapsim_config.domain)
    scenario = pddl.load_scenario(global_vars.mapsim_config.scenario, domain)

    log.info("settings: %s", repr(global_vars.mapsim_config))

    print "Loaded scenario '%s'." % scenario.name
    if len(scenario.agents) == 1:
        print "There is one agent: %s" % "".join(scenario.agents.iterkeys())
    else:
        print "There are %d agents: %s" % (len(scenario.agents), ", ".join(scenario.agents.iterkeys()))

    print "Goals are"
    for agt, prob in scenario.agents.iteritems():
        print "  %s: %s" % (agt, prob.goal.pddl_str())

    if global_vars.mapsim_config.agent_type:
        elems = global_vars.mapsim_config.agent_type.split(".")
        module_elems = elems[:-1]
        if module_elems:
            exec("import %s" % ".".join(module_elems))
        exec("agent_type = %s" % global_vars.mapsim_config.agent_type)
    else:
        agent_type = agent.Agent

    sim = simulation.Simulation(scenario, global_vars.mapsim_config.runs, agent_class=agent_type)

    print "Starting simulation..."
    # cProfile.run('sim.run()', 'mapsim_profile')
    sim.run()
    
    if global_vars.mapsim_config.runs > 1:
        print "\nAverage stats:"
        print "Stats:", sim.collect_average_statistics().sorted_repr(global_vars.mapsim_config.stat_order.split())

    log.info("runs: %d", global_vars.mapsim_config.runs)
    log.info("stats: %s", sim.collect_average_statistics().sorted_repr(global_vars.mapsim_config.stat_order.split()))

