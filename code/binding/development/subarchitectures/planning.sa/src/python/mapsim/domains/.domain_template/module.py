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
        #reporter.set_verb_complement("move", "$arg0 to $arg1")
        pass
