#! /usr/bin/env python2.5

"""Unit tests for all MAPL related stuff"""

# import stdlib modules
import unittest
import sys, os, os.path

# import some standard mapsim modules
import support
import config

# import modules specific for these tests
PLANNING_DIR = os.path.join(support.MAPSIM_DIR, "planning")
sys.path[1:1] = [PLANNING_DIR]

import omniORB
from omniORB import CORBA, PortableServer
from Planner__POA import PlannerServer as PlannerInterface
from Planner import MonitorOutput

os.chdir(support.TEST_DIR)

class BasicTests(unittest.TestCase):
    def setUp(self):
        pi = PlannerInterface()
        import Planner_idl
#         print dir(pi)
#         print dir(Planner_idl)
#         print dir(Planner_idl._0_Planner)
#         print dir(Planner_idl._0_Planner.PlannerServer)
#         print dir(MonitorOutput)
#         mo = MonitorOutput(1,2,3)
#         print dir(mo)
#         print mo._is_a(MonitorOutput)
        
    def testInterface(self):
        pass

if __name__ == "__main__":
	unittest.main()
