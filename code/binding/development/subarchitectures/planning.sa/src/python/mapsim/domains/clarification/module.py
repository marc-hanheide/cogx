# stdlib imports
import os.path
import sys

# mapsim imports
import simulation
import agent


class ClarificationAgent(agent.Agent):
    pass


class Simulation(simulation.Simulation):

    agent_class = ClarificationAgent

