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
        reporter.set_verb_complement("grasp", "the $arg0")
        reporter.set_verb_complement("drop", "the $arg0 in $arg1")
        reporter.set_verb_complement("give", "$arg1 the $arg0")
        reporter.set_verb_complement("get_A", "the $arg0")
        reporter.set_verb_complement("bring_A", "$arg0 the $arg1")
        reporter.set_verb_complement("open", "the $arg0")
        reporter.set_template("get_A", "do", "get")
        reporter.set_template("bring_A", "do", "bring")
        reporter.set_template("ask_for_goal", "DOES_TMPL", "$speaker: 'What can I do for you, $arg0?'")
        reporter.set_template("pos", "question", "Where is the $svarg0")
        reporter.set_template("pos", "answer", "The $svarg0 is in the $value")

