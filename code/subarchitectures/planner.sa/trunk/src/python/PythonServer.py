import os, sys, string, traceback, Ice
from os.path import abspath, dirname, join, isdir
from collections import defaultdict

# initialize logging EARLY

import cast.pylog4cxx

cast_log4cxx = None

class CASTLoggerProxy(object):
    def __init__(self, name=None):
        self.name = name
        self.log = None
        self.cast_id = None
        self.connect()

    def connect(self):
        if self.log and self.cast_id == cast_log4cxx.id:
            return
        if cast_log4cxx:
            if self.name:
                name = "%s.%s" % (cast_log4cxx.id, self.name)
            else:
                name = cast_log4cxx.id
            self.log = cast.pylog4cxx.Logger(name, cast_log4cxx.subarch, cast_log4cxx.color)
            self.cast_id = cast_log4cxx.id

    def log(self, level, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.info(msg)

    def debug(self, msg, *args):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
           args = args[0]
        if args:
           msg = msg % args
            
        self.log.debug(msg)

    def info(self, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
           args = args[0]
        if args:
           msg = msg % args

        self.log.info(msg)

    def warning(self, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.warn(msg)
        
    def error(self, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.error(msg)

    def critical(self, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.fatal(msg)

from standalone import config

config.set_logging_factory(CASTLoggerProxy)

import beliefs_cast_ice
import beliefs_ice
#import de.dfki.lt.tr.beliefs.slice ## must be imported *before* Planner
from autogen import Planner
import cast.core

import standalone
from standalone import pddl, plans
from standalone.pddl import state

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

from cast_task import CASTTask

this_path = abspath(dirname(__file__))

def extend_pythonpath():
    """add standalone planner to PYTHONPATH"""
    standalone_path = join(this_path, "standalone")
    sys.path.insert(0, standalone_path)
    
extend_pythonpath()  

TEST_DOMAIN_FN = join(dirname(__file__), "domains/springtest.mapl")

log = config.logger()
        
class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  
  def __init__(self):
    cast.core.CASTComponent.__init__(self)
    self.domain_fn = TEST_DOMAIN_FN
    self.client = None
    self.dt = None
    self.planner = StandalonePlanner()
    self.tasks = {}
    self.beliefs = None

    self.dttasks = {}
    self.max_dt_id = 0

    global cast_log4cxx
    cast_log4cxx = self.m_logger

    log.info("created new PythonServer")

  def configureComponent(self, config):
    log.info("registering Python planner server")
    self.registerIceServer(Planner.PythonServer,self)
    log.debug("and trying to get it back again immediately")
    server = self.getIceServer("PlannerPythonServer", Planner.PythonServer, Planner.PythonServerPrx)
    log.debug("It worked. We got a server: %s", str(server))
    
    self.client_name = config.get("--wm", "Planner")
    self.dt_name = config.get("--dt", "PlannerDTServer")
    self.show_dot = "--nodot" not in config

    if "--dtdomain" in config:
        self.dtdomain_fn = join(dirname(__file__), "dtdomains", config["--dtdomain"])
    else:
        self.dtdomain_fn = None
        
    if "--dtproblem" in config:
        self.dtproblem_fn = join(dirname(__file__), "dtdomains", config["--dtproblem"])
    else:
        self.dtproblem_fn = None
    
    if "--domain" in config:
      self.domain_fn = join(dirname(__file__), "domains", config["--domain"])

  def getClient(self):
    if not self.client:
      self.client = self.getIceServer(self.client_name, Planner.CppServer, Planner.CppServerPrx)
      log.info("Connected to CppServer %s", self.client_name)
    return self.client

  def getDT(self):
    if not self.dt:
      self.dt = self.getIceServer(self.dt_name, Planner.DTPServer, Planner.DTPServerPrx)
      log.info("Connected to DTPServer %s", self.dt_name)
    return self.dt
  
  def startComponent(self):
    global cast_log4cxx
    cast_log4cxx = self.m_logger

  def stopComponent(self):
    pass

  def runComponent(self):
    pass

  def registerTask(self, task_desc, current=None):
    # MB: id?
    log.info("Planner PythonServer: New PlanningTask received:")
    #log.info("GOAL: %s", task_desc.goal)

    # test the DT interface
    if self.dtdomain_fn and self.dtproblem_fn:
        self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);
        for i in xrange(0, 3):
            tid = self.max_dt_id
            self.max_dt_id += 1
            
            task = Task(tid)
            task.dt_orig_id = task_desc.id
            self.tasks[task_desc.id] = task
            self.dttasks[tid] = task
            task.dt_calls = 1
            log.info("%d: Calling DT planner with problem '%s' and domain '%s'", tid, self.dtproblem_fn, self.dtdomain_fn)
            self.getDT().newTask(tid, self.dtproblem_fn, self.dtdomain_fn);
            log.info("%d: done", tid)
        return

    #task = self.create_task(task_desc, self.domain_fn)
    task = CASTTask(task_desc, self.beliefs, self.domain_fn)
    self.tasks[task.id] = task

    self.planner.register_task(task.cp_task)
    self.getClient().updateStatus(task.id, Planner.Completion.INPROGRESS);

    problem_fn = abspath(join(this_path, "problem%d.mapl" % task.id))
    task.write_cp_problem(problem_fn)
    
    task.replan()
    self.deliver_plan(task)

  # def create_task(self, cast_task, domain_fn):
  #     task = Task(cast_task.id)
  #     task.load_mapl_domain(domain_fn)
      
  #     cast_state = CASTState(self.beliefs, task.mapldomain)
  #     task.mapltask = cast_state.to_problem(cast_task, deterministic=True)
  #     task.cast_state = cast_state
  #     task.set_state(cast_state.state)

  #     log.debug(str(task.get_state()))
  #     return task
    
  def deliver_plan(self, task):    
    plan = task.get_plan()
    
    if not plan:
      self.getClient().updateStatus(task.id, Planner.Completion.FAILED);
      return

    if task.dt_planning_active():
        self.start_dt_planning(task)
        return

    log.debug("The following plan was found %s:\n", plan)

    G = plan.to_dot()
    dot_fn = abspath(join(this_path, "plan%d.dot" % task.id))
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
      
      uargs = [task.state.featvalue_from_object(arg) for arg in pnode.args]

      fullname = str(pnode)
      outplan.append(Planner.Action(task.id, pnode.action.name, uargs, fullname, Planner.Completion.PENDING))
      #outplan.append(Planner.Action(task_desc.id, pnode.action.name, pnode.args, fullname, Planner.Completion.PENDING))
    #print [a.fullName for a in  outplan]
    if outplan:
      log.info("First action: %s == %s", str(ordered_plan[first_action]), outplan[0].fullName)
    else:
      log.info("Plan is empty")
    plan.execution_position = first_action
    
    self.getClient().deliverPlan(task.id, outplan);

  def updateState(self, state, current=None):
      log.debug("recieved state update.")
      self.beliefs = state
      print "state:"
      for bel in state:
          print bel.id
      
  
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

    finished_actions = []
    failed_actions = []
    
    if plan is None:
      #always replan if we don't have a plan
      task.mark_changed()
      
    elif task.dt_planning_active():
        assert len(task_desc.plan) == 1
        dt_action = task_desc.plan[0]

        #Failed dt action causes all actions resulting in the subplan to fail
        if dt_action.status == Planner.Completion.ABORTED or Planner.Completion.FAILED:
            for pnode in task.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
                failed_actions.append(pnode)
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

      if finished_actions or failed_actions:
          diffstate = compute_state_updates(task.cp_state.get_state(), finished_actions, failed_actions)
          for fact in diffstate.iterfacts():
              task.cp_task.get_state().set(fact)
          #TODO: create new state?
          beliefs = task.state.update_beliefs(diffstate)
          self.getClient().updateBeliefState(beliefs)
        
      self.monitor_task(task)

  def start_dt_planning(self, task):
      self.getClient().updateStatus(task.id, Planner.Completion.INPROGRESS);
      
      planning_tmp_dir =  standalone.globals.config.tmp_dir
      tmp_dir = standalone.planner.get_planner_tempdir(planning_tmp_dir)
      domain_fn = os.path.join(tmp_dir, "domain.dtpddl")
      problem_fn = os.path.join(tmp_dir, "problem.dtpddl")
      
      task.dt_task.write_dt_input(domain_fn, problem_fn)
      self.getDT().newTask(task.id, problem_fn, domain_fn);

  def monitor_dt(self, task):
      #test if the dt goals are satisfied:
      sat = True
      for pnode in task.dt_task.subplan_actions:
          if any(fact not in task.state.state for fact in pnode.effects):
              sat = False
              break
      if sat:
          for pnode in task.dt_task.subplan_actions:
              pnode.status == plans.ActionStatusEnum.EXECUTED
          return False

      obs = Planner.Observation("predicate", ["arg1", "arg2"])
      self.getDT().deliverObservation(taskId, [obs])
      log.info("%d: delivered dummy observation", taskId)

      return True
  
  def monitor_task(self, task):
    old_state = task.cp_task.get_state()

    if not task.update_task(self.beliefs):
        print_state_difference(old_state, task.state.state)
        self.deliver_plan(task)
        return

    if task.dt_planning_active():
        if self.monitor_dt(task):
            return
    
    print_state_difference(old_state, task.state.state)
      
    self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);

    problem_fn = abspath(join(this_path, "problem%d.mapl" % task.id))
    f = open(problem_fn, "w")
    f.write(task.cp_task.problem_str(pddl.mapl.MAPLWriter))
    f.close()

    task.replan()
    self.deliver_plan(task)
      

  def deliverAction(self, taskId, action, current=None):
    if taskId not in self.tasks:
      log.warning("Warning: received action for task %d, but no such task found.", taskId)
      return
    
    log.info("%d: received new action from DT", taskId)
      
    task = self.tasks[taskId]

    #create featurevalues
    uargs = [task.state.featvalue_from_object(task.cp_task.mapltask[arg]) for arg in action.arguments]
    
    fullname = action.name + " ".join(action.arguments)
    outplan = [Planner.Action(task.id, action.name, uargs, fullname, Planner.Completion.PENDING)]

    log.info("%d: First action: %s", taskId, fullname)

    # obs = Planner.Observation("predicate", ["arg1", "arg2"])
    
    # self.getDT().deliverObservation(taskId, [obs])

    # log.info("%d: delivered dummy observation", taskId)
    
    self.getClient().deliverPlan(task.id, outplan);

  def updateStatus(self, taskId, status, message, current=None):
      if taskId not in self.dttasks:
          log.warning("Warning: received state update for task %d, but no such task found.", taskId)
          return

      log.info("DT planner updates status to %s with the following message: %s", str(status), message)
      
      #just forward the status for now
      self.getClient().setStatus(self.dttasks[taskId].dt_orig_id, status);


def compute_state_updates(_state, actions, failed):
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
                             
