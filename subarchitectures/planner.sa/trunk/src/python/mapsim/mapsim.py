import simulation
import sys

from standalone import mapl_new as mapl

if __name__ == '__main__':    
    assert len(sys.argv) == 3, """Call 'mapsim.py domain.mapl scenario.mapl' for a single run"""
    domain_fn, scenario_fn = sys.argv[1:]

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
    
