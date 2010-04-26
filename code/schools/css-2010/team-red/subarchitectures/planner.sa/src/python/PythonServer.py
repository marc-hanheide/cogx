import os, sys, logging, traceback, Ice
from os.path import abspath, dirname, join, isdir
from collections import defaultdict

LOG_FORMAT="[%(levelname)s %(name)s: %(message)s]"
logging.basicConfig(format=LOG_FORMAT)

import autogen.Planner as Planner
import beliefmodels.autogen as bm
from beliefmodels.autogen import distribs, featurecontent
#import binder.autogen.core
import cast.core
import cast.cdl
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

BINDER_SA = "binder"

TEST_DOMAIN_FN = join(dirname(__file__), "domains/springtest.mapl")

class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  
  def __init__(self):
    cast.core.CASTComponent.__init__(self)
    self.domain_fn = TEST_DOMAIN_FN
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

    if "--domain" in config:
      self.domain_fn = join(dirname(__file__), "domains", config["--domain"])

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
    task = task_preprocessor.generate_mapl_task(task_desc, self.domain_fn)
    self.tasks[task_desc.id] = task

    self.planner.register_task(task)
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    problem_fn = abspath(join(this_path, "problem%d.mapl" % task.taskID))
    f = open(problem_fn, "w")
    f.write(task.problem_str(pddl.mapl.MAPLWriter))
    f.close()
    
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
      
      uargs = [featvalue_from_object(arg, task.namedict, task.beliefdict) for arg in pnode.args]

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

    # for bel in task_desc.state:
    #   print bel

    finished_actions = []
    failed_actions = []
    if plan is None:
      #always replan if we don't have a plan
      task.mark_changed()
    else:
      log.debug("checking execution state")
      executable_plan = plan.topological_sort()[plan.execution_position:-1]
        
      if len(task_desc.plan) != len(executable_plan):
        log.error("wm plan:")
        for action in task_desc.plan:
          log.error("%s, status: %s", action.fullName, str(action.status))
        log.error("internal plan (execution position is %d):", plan.execution_position)
        for pnode in plan.topological_sort():
          log.error("%s, status: %s", str(pnode), pnode.status)
        raise Exception("Plans from WMControl and Planner don't match!")

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
            failed_actions.append(pnode)
            task.mark_changed()
            
      if requires_action_dispatch:
        task.mark_changed()
    
    import task_preprocessor
    objects, new_state = task_preprocessor.generate_mapl_state(task_desc, task)
    #print map(str, objects)
    #print map(str, facts)

    old_state = task.get_state()
    print_state_difference(old_state, new_state)

    newtask = pddl.Problem(task.mapltask.name, objects, [], None, task._mapldomain, task.mapltask.optimization, task.mapltask.opt_func)

    #check if the goal is still valid
    try:
      newtask.goal = task._mapltask.goal.copy(newtask)
    except KeyError:
      newtask.goal = pddl.conditions.Falsity()

    new_state.problem = newtask
    task.set_state(new_state)
    task.mapltask = newtask

    if finished_actions:
      diffstate = compute_state_updates(task.get_state(), old_state, finished_actions, failed_actions)
      for fact in diffstate.iterfacts():
        task.get_state().set(fact)
      beliefs = update_beliefs(diffstate, task.namedict, task_desc.state)
      self.getClient().updateBeliefState(beliefs)
      
    #print "\n".join(mapl.writer.MAPLWriter().write_problem(newtask))
  
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    problem_fn = abspath(join(this_path, "problem%d.mapl" % task.taskID))
    f = open(problem_fn, "w")
    f.write(task.problem_str(pddl.mapl.MAPLWriter))
    f.close()

    task.replan()
    self.deliver_plan(task)

def compute_state_updates(_state, old_state, actions, failed):
  diffstate = state.State()
  for action in actions:
    for fact in action.effects:
      if fact.svar.modality != pddl.mapl.update:
        continue

      fact = state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])

      if fact not in _state:
        diffstate.set(fact)
        log.debug("not in state: %s", str(fact))
      elif fact.svar in diffstate:
        del diffstate[fact.svar]
        log.debug("previous change %s overwritten by later action", str(state.Fact(fact.svar, diffstate[fact.svar])))

  for action in failed:
    for fact in action.effects:
      if fact.svar.modality != pddl.mapl.update_fail:
        continue

      fact = state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])

      if fact not in _state:
        diffstate.set(fact)
        log.debug("not in state: %s", str(fact))
      elif fact.svar in diffstate:
        del diffstate[fact.svar]
        log.debug("previous change %s overwritten by later action", str(state.Fact(fact.svar, diffstate[fact.svar])))
        
  return diffstate

def featvalue_from_object(arg, namedict, bdict):
  if arg in namedict:
    #arg is provided by the binder
    name = namedict[arg]
  else:
    #arg is a domain constant
    name = arg.name

  if name in bdict:
    #arg is a pointer to another belief
    value = featurecontent.PointerValue(cast.cdl.WorkingMemoryAddress(name, BINDER_SA))
  elif arg.is_instance_of(pddl.t_boolean):
    value = False
    if arg == pddl.TRUE:
      value = True
    value = featurecontent.BooleanValue(value)
  else:
    #assume a string value
    value = featurecontent.StringValue(name)

  return value

def update_beliefs(diffstate, namedict, beliefs):
  bdict = dict((b.id, b) for b in beliefs)
  changed_ids = set()
  new_beliefs = []

  def get_value_dist(dist, feature):
    if isinstance(dist, distribs.DistributionWithExistDep):
      return get_feature_dist(dist.Pc, feature)
    if isinstance(dist, distribs.CondIndependentDistribs):
      if feature in dist.distribs:
        return dist.distribs[feature].values, dist
      return None, dist
    if isinstance(dist, distribs.BasicProbDistribution):
      if dist.key == feature:
        return dist.values, None
      return None, None
    assert False, "class %s not supported" % str(type(dist))

  def find_relation(args, beliefs):
    result = None

    arg_values = [featvalue_from_object(arg, namedict, bdict) for arg in args]
    
    for bel in beliefs:
      if not bel.type == "relation":
        continue
      
      found = True
      for i, arg in enumerate(arg_values):
        dist, _ = get_value_dist(bel.content, "element%d" % i)
        if not dist:
          found = False
          break

        elem = dist.values[0].val
        if elem != arg:
          found = False
          break

      if found:
        result = bel
        break

    if result:
      return result

    frame = bm.framing.SpatioTemporalFrame()
    eps = bm.epstatus.PrivateEpistemicStatus("robot")
    hist = bm.history.CASTBeliefHistory([], [])
    dist_dict = {}
    for i, arg in enumerate(arg_values):
      feat = "element%d" % i
      pair = distribs.FeatureValueProbPair(arg, 1.0)
      dist_dict[feat] = distribs.BasicProbDistribution(feat, distribs.FeatureValues([pair])) 
    dist = distribs.CondIndependentDistribs(dist_dict)
    result = bm.beliefs.StableBelief(frame, eps, "temporary", "relation", dist, hist)
    return result
    
  for svar, val in diffstate.iteritems():
    if len(svar.args) == 1:
      obj = svar.args[0]
      try:
        bel = bdict[namedict[obj]]
      except:
        log.warning("tried to find belief for %s, but failed", str(obj))
        continue
    else:
      bel = find_relation(svar.args, beliefs)
      if bel.id == "temporary":
        beliefs.append(bel)
        new_beliefs.append(bel)
      
    feature = svar.function.name
    dist, parent = get_value_dist(bel.content, feature)
    if not dist:
      if isinstance(parent, distribs.CondIndependentDistribs):
        dist = distribs.FeatureValues()
        parent.distribs[feature] = distribs.BasicProbDistribution(feature, dist)
      else:
        continue
      
    #TODO: deterministic state update for now
    if isinstance(dist, distribs.NormalValues):
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
          fval = featurecontent.PointerValue(cast.cdl.WorkingMemoryAddress(name, BINDER_SA))
        else:
          fval = featurecontent.StringValue(name)
          
      pair = distribs.FeatureValueProbPair(fval, 1.0)
      dist.values = [pair]

    if bel.id != "temporary":
      changed_ids.add(bel.id)

  return [bdict[id] for id in changed_ids] + new_beliefs
  
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
                             
