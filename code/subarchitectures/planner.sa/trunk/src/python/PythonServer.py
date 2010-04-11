import os, sys, logging, traceback, Ice
from os.path import abspath, dirname, join, isdir
from collections import defaultdict

import autogen.Planner as Planner
from beliefmodels.autogen import distribs, featurecontent
#import binder.autogen.core
import cast.core
from standalone import pddl, plans, config
from standalone.pddl import state

log = config.logger("PythonServer")

this_path = abspath(dirname(__file__))

def extend_pythonpath():
  """add standalone planner to PYTHONPATH"""
  standalone_path = join(this_path, "standalone")
  sys.path.insert(0, standalone_path)
extend_pythonpath()  

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner


TEST_DOMAIN_FN = join(dirname(__file__), "test_data/minidora.domain.mapl")
class DummyWriter(object):
  def write(*args):
    pass

class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  
  def __init__(self):
    cast.core.CASTComponent.__init__(self)
    self.client = None
    self.planner = StandalonePlanner()
    self.tasks = {}
    log.info("created new PythonServer")

  def configureComponent(self, config):
    log.info("registering Python planner server")
    self.registerIceServer(Planner.PythonServer,self)
    log.debug("and trying to get it back again immediately")
    server = self.getIceServer("PlannerPythonServer", Planner.PythonServer, Planner.PythonServerPrx)
    log.debug("It worked. We got a server: %s", str(server))
    
    self.client_name = config.get("--wm", "Planner")
    self.show_dot = "--nodot" not in config

    logging.getLogger().setLevel(logging.WARNING)
    if "--log" in config:
      logging.getLogger().setLevel(logging.INFO)
    if "--debug" in config:
      logging.getLogger().setLevel(logging.DEBUG)


  def getClient(self):
    if not self.client:
      self.client = self.getIceServer(self.client_name, Planner.CppServer, Planner.CppServerPrx)
      log.info("Connected to CppServer %s", self.client_name)
    return self.client

  def startComponent(self):
    pass

  def stopComponent(self):
    pass

  def runComponent(self):
    pass

  def registerTask(self, task_desc, current=None):
    # MB: id?
    log.info("Planner PythonServer: New PlanningTask received:")
    log.info("GOAL: %s", task_desc.goal)
    #print "OBJECTS: " + task_desc.objects;
    #print "INIT: " + task_desc.state;

    import task_preprocessor
    task = task_preprocessor.generate_mapl_task(task_desc, TEST_DOMAIN_FN)
    self.tasks[task_desc.id] = task

    self.planner.register_task(task)
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    task.replan()
    self.deliver_plan(task)
    
  def deliver_plan(self, task):    
    if task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
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
    log.debug("The following plan was found %s:\n", plan)

    G = plan.to_dot()
    dot_fn = abspath(join(this_path, "plan%d.dot" % task.taskID))
    G.write(dot_fn)
    log.debug("Dot file for plan is stored in %s", dot_fn)
    
    if self.show_dot:
      log.info("Showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh")
      show_dot_script = abspath(join(this_path, "show_dot.sh"))
      os.system("%s %s" % (show_dot_script, dot_fn)) 
    
    ordered_plan = plan.topological_sort()
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
    #print [a.fullName for a in  outplan]
    if outplan:
      log.info("First action: %s == %s", str(ordered_plan[first_action]), outplan[0].fullName)
    else:
      log.info("Plan is empty")
    plan.execution_position = first_action
    
    self.getClient().deliverPlan(task.taskID, outplan);

  
  def updateTask(self, task_desc, current=None):
    if task_desc.id not in self.tasks:
      log.warning("Warning: received update for task %d, but no such task found.", task_desc.id)
      return
    
    log.info("received task update")
    if task_desc.executionStatus in (Planner.Completion.SUCCEEDED, Planner.Completion.FAILED, Planner.Completion.ABORTED):
      log.info("task %d is done.", task_desc.id)
      del self.tasks[task_desc.id]
      return
      
    task = self.tasks[task_desc.id]
    plan = task.get_plan()

    if plan is None:
      #always replan if we don't have a plan
      task.mark_changed()
    else:
      log.debug("checking execution state")
      executable_plan = plan.topological_sort()[plan.execution_position:-1]
        
      if len(task_desc.plan) != len(executable_plan):
        for action in task_desc.plan:
          log.error("%s, status: %s", action.fullName, str(action.status))
        for pnode in plan.topological_sort():
          log.error("%s, status: %s", str(pnode), pnode.status)
        raise Exception("Plans from WMControl and Planner don't match!")

      finished_actions = []
      requires_action_dispatch = False
      for action, pnode in zip(task_desc.plan, executable_plan):
        if action.status != Planner.Completion.PENDING:
          log.debug("status of %s is %s", action.fullName, str(action.status))
          
        if action.status != Planner.Completion.PENDING:
          if action.status == Planner.Completion.INPROGRESS:
            requires_action_dispatch = False
            pnode.status = plans.ActionStatusEnum.IN_PROGRESS
          elif action.status == Planner.Completion.SUCCEEDED:
            requires_action_dispatch = True
            finished_actions.append(pnode)
            pnode.status = plans.ActionStatusEnum.EXECUTED
          elif action.status == Planner.Completion.ABORTED or Planner.Completion.FAILED:
            pnode.status = plans.ActionStatusEnum.FAILED
            task.mark_changed()
            
      if requires_action_dispatch:
        task.mark_changed()
    
    import task_preprocessor
    objects, facts = task_preprocessor.generate_mapl_state(task_desc, task)
    #print map(str, objects)
    #print map(str, facts)

    print_state_difference(task.get_state(), state.State(facts))

    newtask = pddl.Problem(task.mapltask.name, objects, [], None, task._mapldomain)

    #check if the goal is still valid
    try:
      newtask.goal = task._mapltask.goal.copy(newtask)
    except KeyError:
      newtask.goal = pddl.conditions.Falsity()
    
    task.set_state(state.State(facts, newtask))
    task.mapltask = newtask

    if finished_actions:
      diffstate = compute_state_updates(task.get_state(), finished_actions)
      beliefs = update_beliefs(diffstate, task.namedict, task_desc.state)
      self.getClient().updateBeliefState(beliefs)
      
    #print "\n".join(mapl.writer.MAPLWriter().write_problem(newtask))
  
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    task.replan()
    self.deliver_plan(task)

def compute_state_updates(state, actions):
  diffstate = state.State()
  for action in actions:
    for fact in action.effects:
      if fact not in state:
        diffstate.set(fact)
        log.debug("not in state: %s", str(fact))
      elif fact.svar in diffstate:
        del diffstate[fact.svar]
        log.debug("previous change %s overwritten by later action", str(state.Fact(fact.svar, diffstate[fact.svar])))

  return diffstate

def update_beliefs(diffstate, namedict, beliefs):
  bdict = dict((b.id, b) for b in beliefs)
  changed_ids = set()

  def get_feature_dist(dist, feature):
    if isinstance(dist, distribs.DistributionWithExistDep):
      return get_feature_dist(dist.Pc, feature)
    if isinstance(dist, distribs.CondIndependentDistribs):
      for d in dist.distribs:
        result = get_feature_dist(d, feature)
        if result:
          return result
      return None
    if isinstance(dist, distribs.FeatureValueDistribution):
      if dist.feat == feature:
        return dist
      return None
    if isinstance(dist, distribs.NormalDistribution):
      if dist.feat == feature:
        return dist
      return None
    if isinstance(dist, distribs.DiscreteDistribution):
      assert False, "DiscreteDistribution not supported yet"
    assert False, "class %s not supported" % str(type(dist))
  
  for svar, val in diffstate.iteritems():
    obj = svar.args[0]
    bel = bdict[namedict[obj]]

    dist = get_feature_dist(bel.content, svar.function.name)
    #TODO: deterministic state update for now
    if isinstance(dist, distribs.NormalDistribution):
      dist.mean = val.value
      dist.variance = 0;
    else:
      if val.is_instance_of(pddl.t_number):
        fval = featurecontent.IntegerValue(val.value)
      elif val.is_instance_of(pddl.t_boolean):
        if val == pddl.TRUE:
          fval = featurecontent.BooleanValue(True)
        else:
          fval = featurecontent.BooleanValue(False)
      elif val == pddl.UNKNOWN:
        fval = featurecontent.UnknownValue()
      else:
        name = namedict[val]
        if ":" in name:
          fval = featurecontent.PointerValue(name)
        else:
          fval = featurecontent.StringValue(name)
          
      pair = distribs.FeatureValueProbPair(fval, 1.0)
      dist.values = [pair]

    changed_ids.add(bel.id)

  return [bdict[id] for id in changed_ids]
  
def print_state_difference(state1, state2, print_fn=None):
  def collect_facts(state):
    facts = defaultdict(set)
    for svar,val in state.iteritems():
      facts[svar.args[0]].add((svar, val))
    return facts

  if not print_fn:
    print_fn = log.debug

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
    print_fn("\nNew objects:")
    for o in new:
      print " %s:" % o.name
      for svar, val in f2[o]:
        print_fn( "    %s = %s", str(svar), str(val))

  if changed:
    print_fn("\nChanged objects:")
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
        print_fn(" %s:", o.name)
        for svar in fnew:
          print_fn("    %s: unknown => %s", str(svar), str(state2[svar]))
        for svar in fchanged:
          print_fn("    %s: %s => %s", str(svar), str(state1[svar]), str(state2[svar]))
        for svar in fremoved:
          print_fn("    %s: %s => unknown", str(svar), str(state1[svar]))
                            
  if removed:
    print_fn("\nRemoved objects:")
    for o in removed:
      print_fn(" %s:", o.name)
      for svar, val in f1[o]:
        print_fn("    %s = %s", str(svar), str(val))
                             
