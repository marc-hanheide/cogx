#! /usr/bin/env python2.5

"""Unit tests for reporter.py"""

# import stdlib modules
import unittest
import os

# import mapsim modules
import support
import config
import reporter
import commands
from dummies import Agent

os.chdir(support.TEST_DIR)

class BasicTests(unittest.TestCase):
    def setUp(self):
        """ provide basic commands to verbalize """
        self.commands = [] #  [ (cmd, expected_report), ...]
        self.reporter = reporter.Reporter()
        dummy_simulator = None
        speaker = Agent(dummy_simulator, "TheSpeaker")
        hearer = Agent(dummy_simulator, "TheHearer")
        anAgent = Agent(dummy_simulator, "TheAgent")
        # Example command: wait
        wait_cmd = commands.WaitCommand(agent=speaker)
        self.commands.append((wait_cmd, "TheSpeaker waits."))
        # Example command: Jump
        no_arg_cmd = commands.PhysicalAction(agent=anAgent, operator="jump")
        self.commands.append((no_arg_cmd, "TheAgent jumps."))

    def testSampleCommands(self):
        """ test some reporting for some specific example commands """
        for cmd, expected_report in self.commands:
            report = self.reporter.report_on(cmd)
            self.assertEqual(report, expected_report)

if __name__ == "__main__":
	unittest.main()
