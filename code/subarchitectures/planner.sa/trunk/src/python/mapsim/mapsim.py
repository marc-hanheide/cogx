#! /usr/bin/env python
# -*- coding: latin-1 -*-

"""MAPSIM: a Multiagent Planning simulation environment.

usage: %prog [options] domain scenario
  -d, --debug : print debug messages to console and logfiles [default: %default]
  -v, --verbosity=VERBOSITY : general verbosity level for console output [default: %default]
  -l, --loglevel=VERBOSITY : general verbosity level for logging output [default: %default]
"""

from os.path import abspath, dirname
import sys, logging

from standalone import mapl_new as mapl
import standalone.config as config
import standalone.globals as global_vars

import simulation

log = config.logger()

mapsim_path = abspath(dirname(__file__))  # where this file resides
CONFIG_FN = "config.ini"
global_vars.mapsim_config = global_vars.load_config_file(CONFIG_FN, cfg_path=mapsim_path)

def parse_command_line():
    # parse options and required arguments from docstring automatically
    from myoptionparse import OptionParser
    parser = OptionParser(from_doc=__doc__)
    parser.set_defaults_from(global_vars.mapsim_config.__dict__)
    options, args = parser.parse_args()
    global_vars.mapsim_config.__dict__.update(options.__dict__)

if __name__ == '__main__':
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
                
    #sample for logging configuration:
    #
    #logging.getLogger("file").setLevel(logging.DEBUG)
    #logging.getLogger("file.planner").setLevel(logging.CRITICAL)
    #logging.getLogger("file.agent.planner").setLevel(logging.CRITICAL)
    
    config.set_logfile("mapsim.log")
    
    config.set_logfile("agent.log", "agent")
    config.set_logfile("agent2.log", "agent2")
    #config.set_logfile("planner.log", "planner")
    #log.info("test")

    domain = mapl.load_domain(global_vars.mapsim_config.domain)
    scenario = mapl.load_scenario(global_vars.mapsim_config.scenario, domain)

    print "Loaded scenario '%s'." % scenario.name
    if len(scenario.agents) == 1:
        print "There is one agent: %s" % "".join(scenario.agents.iterkeys())
    else:
        print "There are %d agents: %s" % (len(scenario.agents), ", ".join(scenario.agents.iterkeys()))

    print "Goals are"
    for agent, prob in scenario.agents.iteritems():
        print "  %s: %s" % (agent, prob.goal.pddl_str())
        
    sim = simulation.Simulation(scenario)
    print "Starting simulation..."
    sim.run()
    
