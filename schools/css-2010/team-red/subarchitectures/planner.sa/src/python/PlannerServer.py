import cast.core
import time



class PlannerServer(cast.core.CASTComponent):
  def __init__(self):
    print "creating planner server"

  def startComponent(self):
    print "starting planner server"

  def stopComponent(self):
    print "stopping planner server"

  def configureComponent(self, _config):
    print "configured planner server"

  def runComponent(self):
    while self.isRunning():
      time.sleep(1)
      print "%s running" % self.getID(None)
