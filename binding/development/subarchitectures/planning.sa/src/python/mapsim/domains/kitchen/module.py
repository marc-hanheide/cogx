# stdlib imports
import os.path
import sys

# mapsim imports
import simulation

class Simulation(simulation.Simulation):
    def setup_reporter_templates(self, reporter):
        reporter.set_verb_complement("bring", "$arg0 the $arg1")
        reporter.set_verb_complement("bake", "the $arg0")
        reporter.set_verb_complement("eat", "the $arg0")
        reporter.set_template("go", "does", "goes")
