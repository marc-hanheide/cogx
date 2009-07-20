import sys, traceback, Ice
import autogen.Planner as Planner
import cast.core

class PythonServerI(Planner.PythonServer, cast.core.CASTComponent):
  def __init__(self):
    self.client = None

  def configure(self):
    pass

  def start(self):
    pass

  def stop(self):
    pass

  def runComponent(self):
    pass

  def addTask(self, task, current=None):
    print "Planner PythonServer: New PlanningTask received"

    if(self.client is None):
      return -1

    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    return 1

  def registerClient(self, Client, current=None):
    print "Planner PythonServer: running"
    self.client = Client
