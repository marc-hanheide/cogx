import sys, traceback, Ice
import autogen.Planner as Planner
import core

class PythonServerI(Planner.PythonServer, core.CASTComponent):
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
    if(self.client is None):
      return -1

    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    return 1

  def registerClient(self, Client, current=None):
    self.client = Client

#status = 0
#ic = None

#try:
#  ic = Ice.initialize(sys.argv)
#  adapter = ic.createObjectAdapterWithEndpoints("SimplePrinterAdapter", "default -p 10000")
#  object = PythonServerI()
#  adapter.add(object, ic.stringToIdentity("SimplePrinter"))
#  adapter.activate()
#  ic.waitForShutdown()
#except:
#  traceback.print_exc()
#  status = 1
#  if ic:
  # Clean up
#    try:
#      ic.destroy()
#    except:
#      traceback.print_exc()
#      status = 1
#      sys.exit(status)
    
