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
from display_client import PlannerDisplayClient

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
    self.m_display = PlannerDisplayClient()

    global cast_log4cxx
    cast_log4cxx = self.m_logger

    log.info("created new PythonServer")

  def configureComponent(self, config):
    log.info("registering Python planner server")
    self.registerIceServer(Planner.PythonServer,self)
    log.debug("and trying to get it back again immediately")
    server = self.getIceServer("PlannerPythonServer", Planner.PythonServer, Planner.PythonServerPrx)
    log.debug("It worked. We got a server: %s", str(server))

    self.m_display.configureDisplayClient(config)
    
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

  def get_path(self):
      return this_path
  
  def startComponent(self):
    global cast_log4cxx
    cast_log4cxx = self.m_logger

  def stopComponent(self):
    pass

  def runComponent(self):
      self.m_display.connectIceClient(self)
      self.m_display.installEventReceiver()

  def registerTask(self, task_desc, current=None):
    log.info("Planner PythonServer: New PlanningTask received:")

    # # test the DT interface
    # if self.dtdomain_fn and self.dtproblem_fn:
    #     self.getClient().updateStatus(task_desc.id, Planner.Completion.INPROGRESS);
    #     for i in xrange(0, 3):
    #         tid = self.max_dt_id
    #         self.max_dt_id += 1
            
    #         task = Task(tid)
    #         task.dt_orig_id = task_desc.id
    #         self.tasks[task_desc.id] = task
    #         self.dttasks[tid] = task
    #         task.dt_calls = 1
    #         log.info("%d: Calling DT planner with problem '%s' and domain '%s'", tid, self.dtproblem_fn, self.dtdomain_fn)
    #         self.getDT().newTask(tid, self.dtproblem_fn, self.dtdomain_fn);
    #         log.info("%d: done", tid)
    #     return

    task = CASTTask(task_desc, self.beliefs, self.domain_fn, self)
    self.tasks[task.id] = task

    task.run()
    
  def deliver_plan(self, task, slice_plan):    
      self.m_display.update_task(task)
      self.getClient().deliverPlan(task.id, slice_plan);
      self.m_display.update_task(task)

  def updateState(self, state, percepts, current=None):
      log.debug("recieved state update.")
      self.beliefs = state
      for task in self.tasks.itervalues():
          task.percepts += percepts
      
  def updateTask(self, task_desc, current=None):
      if task_desc.id not in self.tasks:
          log.warning("Warning: received update for task %d, but no such task found.", task_desc.id)
          return
      
      log.info("received task update")
      if task_desc.executionStatus in (Planner.Completion.SUCCEEDED, Planner.Completion.FAILED, Planner.Completion.ABORTED):
          log.info("task %d is done.", task_desc.id)
          self.m_display.remove_task(self.tasks[task_desc.id])
          del self.tasks[task_desc.id]
          return
        
      task = self.tasks[task_desc.id]

      if not task.update_state(self.beliefs):
          print_state_difference(old_state, task.state.state)
          task.update_status(Planner.Completion.FAILED)
          self.deliver_plan(task, [])
          return

      print_state_difference(old_state, task.state.state)

      if task.dt_planning_active():
          task.action_executed_dt(task_desc.plan)
      else:
          task.action_executed_cp(task_desc.plan)

    
  def start_dt_planning(self, task):
      planning_tmp_dir =  standalone.globals.config.tmp_dir
      tmp_dir = standalone.planner.get_planner_tempdir(planning_tmp_dir)
      domain_fn = os.path.join(tmp_dir, "domain.dtpddl")
      problem_fn = os.path.join(tmp_dir, "problem.dtpddl")
      
      task.dt_task.write_dt_input(domain_fn, problem_fn)
      self.getDT().newTask(task.id, problem_fn, domain_fn);

      
  def deliverAction(self, taskId, action, current=None):
      if taskId not in self.tasks:
          log.warning("Warning: received action for task %d, but no such task found.", taskId)
          return
    
      log.info("%d: received new action from DT", taskId)
      
      task = self.tasks[taskId]
      task.action_delivered(action)

      
  def updateStatus(self, taskId, status, message, current=None):
      if taskId not in self.dttasks:
          log.warning("Warning: received state update for task %d, but no such task found.", taskId)
          return

      log.info("DT planner updates status to %s with the following message: %s", str(status), message)
      task = self.tasks[taskId]

      #notify the cp planner that the subtask is finished
      if status == Planner.Completion.SUCCEEDED:
          task.dt_done()
          return
      
      #just forward the status for now
      self.getClient().updateStatus(task.id, status)

  
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
                             
