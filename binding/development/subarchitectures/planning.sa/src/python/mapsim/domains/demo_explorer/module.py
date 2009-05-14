# stdlib imports
import os.path
import sys

# mapsim imports
import simulation
import agent


class ExplorerAgent(agent.Agent):
    pass


class Simulation(simulation.Simulation):
    agent_class = ExplorerAgent

    def setup_reporter_templates(self, reporter):
        reporter.set_verb_complement("move", "to $arg0")
        reporter.set_template("pos", "question", "Where is the $svarg0")
        reporter.set_template("pos", "answer", "The $svarg0 is in the $value")
   
        reporter.set_template("object-search-in-room", "does", "searches")
        reporter.set_verb_complement("object-search-in-room", "for $arg0 in $arg1")

