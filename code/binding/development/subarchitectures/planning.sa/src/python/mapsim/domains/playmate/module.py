# stdlib imports
import os.path
import sys

# mapsim imports
import simulation
import agent


class PlaymateAgent(agent.Agent):
    pass


class Simulation(simulation.Simulation):
    agent_class = PlaymateAgent

    def setup_reporter_templates(self, reporter):
        reporter.set_verb_complement("move", "$arg0 to $arg1")
        reporter.set_verb_complement("pick_up", "$arg0")
        reporter.set_verb_complement("place", "$arg0 on $arg1")
        reporter.set_verb_complement("give", "$arg1 to $arg0")
