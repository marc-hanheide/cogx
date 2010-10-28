# Standard library imports
import time
from os.path import join, abspath, dirname
# CAST imports
from cast.core import CASTComponent
# Local imports
from cast_wrappers import Task, Planner

# the ICE definitions (as produced by slice2py):
from autogen.Planner import PlanningTask as TaskIce
from autogen.Planner import PlannerServer as PlannerIce
from autogen.Planner import PlannerServerPrx


src_path = abspath(dirname(__file__))
tests_path = abspath(join(src_path, "../../test_data"))

class PlannerTester(cast.core.CASTComponent):
    def __init__(self):
        print "Creating planner tester"

    def startComponent(self):
        print "starting planner tester"

    def stopComponent(self):
        print "stopping planner tester"

    def configureComponent(self, _config):
        print "configured planner tester"

    def runComponent(self):
        print dir(self)
        self.getComponentManager()


    

def setup_test_planner():
    p =  Planner()
    return p

def setup_test_task(planner):
    # prepare test data
    from os.path import join
    domain_file = join(tests_path, "cp_test.domain.mapl")
    task_file = join(tests_path, "cp_test.task.mapl")
    agent_name = "R2D2"
    # create new task
    t = planner.newTask()
    # fill task with some test data
    t.loadMAPLTask(task_file, domain_file, agent_name)
    return t

def main(planner=None):
    if planner is None:
        planner = setup_test_planner()
    planner.printString("Hello, world")
    task = setup_test_task(planner)
    planner.registerTask(task)
    task.markChanged()
    task.activateChangeDetection()


if __name__ == "__main__":
#     import sys, traceback, Ice
#     status = 0
#     ic = None
#     try:
#         ic = Ice.initialize(sys.argv)
#         base = ic.stringToProxy("SimplePlanner:default -p 10000")
#         planner = PlannerServerPrx.checkedCast(base)
#         if not planner:
#             raise RuntimeError("Invalid proxy")
#         main(planner)
#     except:
#         traceback.print_exc()
#         status = 1
#     if ic:
#         # Clean up
#         try:
#             ic.destroy()
#         except:
#             traceback.print_exc()
#             status = 1
#             sys.exit(status)

    pt = PlannerTester()
    main()
    
                                        
