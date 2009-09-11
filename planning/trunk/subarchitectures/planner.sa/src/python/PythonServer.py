import os, sys, traceback, Ice
from os.path import abspath, dirname, join, isdir

import autogen.Planner as Planner
import binder.autogen.core
import cast.core

this_path = abspath(dirname(__file__))

def extend_pythonpath():
  """add standalone planner to PYTHONPATH"""
  standalone_path = join(this_path, "standalone")
  sys.path.insert(0, standalone_path)
extend_pythonpath()  

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner


TEST_DOMAIN_FN = join(dirname(__file__), "test_data/minidora.domain.mapl")

class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  def __init__(self):
    cast.core.CASTComponent.__init__(self)
    self.client = None
    self.planner = StandalonePlanner()
    print "created new PythonServer"

  def configureComponent(self, config):
    print "registering Python planner server"
    self.registerIceServer(Planner.PythonServer,self)
    print "and trying to get it back again immediately"
    server = self.getIceServer("PlannerPythonServer", Planner.PythonServer, Planner.PythonServerPrx)
    print "It worked. We got a server:", server
    self.client_name = config.get("--wm", "Planner")

  def getClient(self):
    if not self.client:
      self.client = self.getIceServer(self.client_name, Planner.CppServer, Planner.CppServerPrx)
      print "Connected to CppServer %s" % self.client_name
    return self.client

  def startComponent(self):
    pass

  def stopComponent(self,config):
    pass

  def runComponent(self):
    pass

  def registerTask(self, task_desc, current=None):
    # MB: id?
    print "Planner PythonServer: New PlanningTask received:"
    print "GOAL: " + task_desc.goal;
    #print "OBJECTS: " + task_desc.objects;
    #print "INIT: " + task_desc.state;

    import task_preprocessor
    task = task_preprocessor.generate_mapl_task(task_desc, TEST_DOMAIN_FN)

    self.planner.register_task(task)
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    task.mark_changed()
    task.activate_change_dectection()
    if task.get_planning_status() == PlanningStatusEnum.PLANNING_FAILURE:
      self.getClient().updateStatus(task_desc.id, Planner.Completion.FAILED);
      return
    
    plan = task.get_plan()
    #plan = task_preprocessor.map2binder_rep(plan, task)
    print "The following plan was found:\n", plan

    dot_str = plan.to_dot()
    dot_fn = abspath(join(this_path, "plan.dot"))
    open(dot_fn, "w").write(dot_str)
    print "Dot file for plan is stored in", dot_fn
    
    show_dot = True
    if show_dot:
      print "Showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh"
      show_dot_script = abspath(join(this_path, "show_dot.sh"))
      os.system("%s %s" % (show_dot_script, dot_fn)) 

    uniondict = dict((u.entityID, u) for u in task_desc.state)
    
    ordered_plan = plan.topological_sort(include_depths=False)
    outplan = []
    for pnode in ordered_plan[1:-1]:
      uargs = [task._mapldomain.namedict.get(a.name, a.name) for a in pnode.args]
      fullname = str(pnode)
      outplan.append(Planner.Action(task_desc.id, pnode.action.name, uargs, fullname, Planner.Completion.PENDING))
      #outplan.append(Planner.Action(task_desc.id, pnode.action.name, pnode.args, fullname, Planner.Completion.PENDING))
    #print outplan
    self.getClient().deliverPlan(task_desc.id, outplan);
                     
    #plan = [Planner.Action(0,"test1",[task_desc.state[0]],Planner.Completion.PENDING), Planner.Action(0,"test2",[],Planner.Completion.PENDING)]
    
    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    
  def updateTask(self, task_desc, current=None):
    pass
