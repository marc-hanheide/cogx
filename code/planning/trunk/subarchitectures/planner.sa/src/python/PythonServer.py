import sys, traceback, Ice
import autogen.Planner as Planner
import cast.core
#import standalone.task

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

  def registerTask(self, task, current=None):
    print "Planner PythonServer: New PlanningTask received:"
    print "GOAL: " + task.goal;
    print "OBJECTS: " + task.objects;
    print "INIT: " + task.state;

    task.plan = "SOMEDAY HERE WILL BE A PLAN!!!"

    if(self.client is None):
      print "ERROR!!"

    self.client.deliverPlan(task);
    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    
  def registerClient(self, Client, current=None):
    print "Planner PythonServer: running"
    self.client = Client
