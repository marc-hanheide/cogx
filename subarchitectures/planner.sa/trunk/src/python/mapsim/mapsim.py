import simulation
import sys

import logging

from standalone import mapl_new as mapl
import standalone.config as config

log = config.logger()

if __name__ == '__main__':
    assert len(sys.argv) == 3, """Call 'mapsim.py domain.mapl scenario.mapl' for a single run"""
    domain_fn, scenario_fn = sys.argv[1:]

    #sample for logging configuration:
    #
    #logging.getLogger().setLevel(logging.INFO)
    logging.getLogger("file").setLevel(logging.DEBUG)
    config.set_logfile("mapsim.log")
    config.set_logfile("agent.log", "agent")
    #config.set_logfile("planner.log", "planner")
    #log.info("test")

    domain = mapl.load_domain(domain_fn)
    scenario = mapl.load_scenario(scenario_fn, domain)

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
    
