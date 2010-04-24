"""
Wrapper classes that act as proxies between the ICE classes defined for CAST
and the stand-alone continual planner that is independent of any particular
application environment.
"""

import sys
from os.path import join, abspath, dirname

# add import path for standalone planner
src_path = abspath(dirname(__file__))
sys.path.insert(0, join(src_path,"standalone"))

# the ICE definitions (as produced by slice2py):
from autogen.Planner import PlanningTask as TaskIce
from autogen.Planner import PlannerServer as PlannerIce

# the relevant classes from the stand-alone planner:
from standalone.task import Task as TaskStandalone
from standalone.planner import Planner as PlannerStandalone


class Task(TaskIce):
    def __init__(self):
        TaskIce.__init__(self)
        self.wrapped_task = TaskStandalone()

    def loadMAPLTask(self, taskFile, domainFile, planningAgent, _ctx=None):
        self.wrapped_task.load_mapl_task(taskFile, domainFile, planningAgent)

    def markChanged(self, _ctx=None):
        self.wrapped_task.mark_changed()

    def activateChangeDetection(self, _ctx=None):
        self.wrapped_task.activate_change_dectection()

    def planAvailable(self, _ctx=None):
        return self.wrapped_task.plan_available()

    def getPlan(self, _ctx=None):
        return self.wrapped_task.get_plan()


class Planner(PlannerIce):
    def __init__(self):
        PlannerIce.__init__(self)
        self.wrapped_planner = PlannerStandalone()

    def newTask(self, _ctx=None):
        task = Task()
        task.id = 0
        return task

    def registerTask(self, task, _ctx=None):
        self.wrapped_planner.register_task(task.wrapped_task)

    def printString(self, astring, _ctx=None):
        print astring


if __name__ == "__main__":
    import sys, traceback, Ice
    status = 0
    ic = None
    try:
        ic = Ice.initialize(sys.argv)
        adapter = ic.createObjectAdapterWithEndpoints("SimplePlannerAdapter", "default -p 10000")
        object = Planner()
        adapter.add(object, ic.stringToIdentity("SimplePlanner"))
        adapter.activate()
        ic.waitForShutdown()
    except:
        traceback.print_exc()
        status = 1
    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1
            sys.exit(status)
