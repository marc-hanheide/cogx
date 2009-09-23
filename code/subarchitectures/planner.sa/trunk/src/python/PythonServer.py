import os, sys, traceback, Ice
from os.path import abspath, dirname, join, isdir
from collections import defaultdict

import autogen.Planner as Planner
import binder.autogen.core
import cast.core
import standalone.mapl_new as mapl
import standalone.state_new as state
import standalone.plans as plans

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
    self.tasks = {}
    print "created new PythonServer"

  def configureComponent(self, config):
    print "registering Python planner server"
    self.registerIceServer(Planner.PythonServer,self)
    print "and trying to get it back again immediately"
    server = self.getIceServer("PlannerPythonServer", Planner.PythonServer, Planner.PythonServerPrx)
    print "It worked. We got a server:", server
    self.client_name = config.get("--wm", "Planner")
    self.show_dot = "--nodot" not in config

  def getClient(self):
    if not self.client:
      self.client = self.getIceServer(self.client_name, Planner.CppServer, Planner.CppServerPrx)
      print "Connected to CppServer %s" % self.client_name
    return self.client

  def startComponent(self):
    pass

  def stopComponent(self):
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
    self.tasks[task_desc.id] = task
    task.set_plan_callback(lambda task: self.getPlan(task))

    self.planner.register_task(task)
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    task.mark_changed()
    task.activate_change_dectection()
    
  def getPlan(self, task):    
    if task.get_planning_status() == PlanningStatusEnum.PLANNING_FAILURE:
      self.getClient().updateStatus(task.taskID, Planner.Completion.FAILED);
      return

    #TEST TEST TEST TEST 
#     import random
#     s = random.random()
#     print s
#     if s > 0.5:
#       task.set_plan(None, update_status=True)
#       return
    #TEST TEST TEST TEST
    
    plan = task.get_plan()
    #plan = task_preprocessor.map2binder_rep(plan, task)
    print "The following plan was found:\n", plan

    dot_str = plan.to_dot()
    dot_fn = abspath(join(this_path, "plan.dot"))
    open(dot_fn, "w").write(dot_str)
    print "Dot file for plan is stored in", dot_fn
    
    if self.show_dot:
      print "Showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh"
      show_dot_script = abspath(join(this_path, "show_dot.sh"))
      os.system("%s %s" % (show_dot_script, dot_fn)) 
    
    ordered_plan = plan.topological_sort(include_depths=False)
    outplan = []
    first_action = -1
    for i,pnode in enumerate(ordered_plan):
      if isinstance(pnode, plans.DummyNode) or not pnode.is_executable():
        continue
      if first_action == -1:
        first_action = i
      
      uargs = [task.namedict.get(a, a.name) for a in pnode.args]

      fullname = str(pnode)
      outplan.append(Planner.Action(task.taskID, pnode.action.name, uargs, fullname, Planner.Completion.PENDING))
      #outplan.append(Planner.Action(task_desc.id, pnode.action.name, pnode.args, fullname, Planner.Completion.PENDING))
    #print outplan
    plan.execution_position = first_action
    
    self.getClient().deliverPlan(task.taskID, outplan);

  
  def updateTask(self, task_desc, current=None):
    if task_desc.id not in self.tasks:
      print "Warning: received update for task %d, but no such task found." % task_desc.id
      return
    
    print "received task update"
    if task_desc.executionStatus in (Planner.Completion.SUCCEEDED, Planner.Completion.FAILED, Planner.Completion.ABORTED):
      print "task %d is done." % task_desc.id
      del self.tasks[task_desc.id]
      return
      
    task = self.tasks[task_desc.id]
    plan = task.get_plan()

    task.suspend_change_dectection()
    if plan is None:
      #always replan if we don't have a plan
      task.mark_changed()
    else:
      print "checking execution state"
      executable_plan = plan.topological_sort(include_depths=False)[plan.execution_position:-1]
        
      if len(task_desc.plan) != len(executable_plan):
        for action in task_desc.plan:
          print "%s, status: %s" % (action.fullName, str(action.status))
        for pnode in plan.topological_sort(include_depths=False):
          print "%s, status: %s" % (str(pnode), pnode.status)
        raise Exception("Plans from WMControl and Planner don't match!")
          
      requires_action_dispatch = False
      for action, pnode in zip(task_desc.plan, executable_plan):
        if action.status != Planner.Completion.PENDING:
          print "status of %s is %s" % (action.fullName, str(action.status))
          
        if action.status != Planner.Completion.PENDING:
          if action.status == Planner.Completion.INPROGRESS:
            requires_action_dispatch = False
            pnode.status = plans.ActionStatusEnum.IN_PROGRESS
          elif action.status == Planner.Completion.SUCCEEDED:
            requires_action_dispatch = True
            pnode.status = plans.ActionStatusEnum.EXECUTED
          elif action.status == Planner.Completion.ABORTED or Planner.Completion.FAILED:
            pnode.status = plans.ActionStatusEnum.FAILED
            
      if requires_action_dispatch:
        task.mark_changed()
    
    import task_preprocessor
    objects, facts = task_preprocessor.generate_mapl_state(task_desc, task)
    #print map(str, objects)
    #print map(str, facts)

    print_state_difference(task.get_state(), state.State(facts))

    newtask = mapl.problem.Problem(task._mapltask.name, objects, [f.asLiteral(useEqual=True) for f in facts], None, task._mapldomain)

    #check if the goal is still valid
    try:
      newtask.goal = task._mapltask.goal.copy(newtask)
    except KeyError:
      newtask.goal = mapl.conditions.Falsity()
    
    task._mapltask = newtask

    #print "\n".join(mapl.writer.MAPLWriter().write_problem(newtask))
  
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    task.set_state(state.State.fromProblem(task._mapltask))
    task.activate_change_dectection()

def print_state_difference(state1, state2):
  def collect_facts(state):
    facts = defaultdict(set)
    for svar,val in state.iteritems():
      facts[svar.args[0]].add((svar, val))
    return facts

  f1 = collect_facts(state1)
  f2 = collect_facts(state2)

  new = []
  changed = []
  removed = []
  for o in f2.iterkeys():
    if o not in f1:
      new.append(o)
    else:
      changed.append(o)
  for o in f1.iterkeys():
    if o not in f2:
      removed.append(o)

  if new:
    print "\nNew objects:"
    for o in new:
      print " %s:" % o.name
      for svar, val in f2[o]:
        print "    %s = %s" % (str(svar), str(val))

  if changed:
    print "\nChanged objects:"
    for o in changed:
      fnew = []
      fchanged = []
      fremoved = []
      for svar,val in f2[o]:
        if svar not in state1:
          fnew.append(svar)
        elif state1[svar] != val:
          fchanged.append(svar)
      for svar,val in f2[0]:
        if svar not in state2:
          fremoved.append(svar)
      if fnew or fchanged or fremoved:
        print " %s:" % o.name
        for svar in fnew:
          print "    %s: unknown => %s" % (str(svar), str(state2[svar]))
        for svar in fchanged:
          print "    %s: %s => %s" % (str(svar), str(state1[svar]), str(state2[svar]))
        for svar in fremoved:
          print "    %s: %s => unknown" % (str(svar), str(state1[svar]))
                            
  if removed:
    print "\nRemoved objects:"
    for o in removed:
      print " %s:" % o.name
      for svar, val in f1[o]:
        print "    %s = %s" % (str(svar), str(val))
                             
