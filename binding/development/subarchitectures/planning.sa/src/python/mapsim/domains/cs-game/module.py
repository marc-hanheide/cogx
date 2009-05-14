# stdlib imports
import os.path
import sys

# mapsim imports
import simulation
import agent


class DummyAgent(agent.Agent):
    pass


class Simulation(simulation.Simulation):
    agent_class = DummyAgent

    def setup_reporter_templates(self, reporter):
         reporter.set_verb_complement("pick_up", "$arg0 from $arg1")
         reporter.set_verb_complement("place", "$arg0 at $arg1")
#         reporter.set_template("go", "does", "goes")
