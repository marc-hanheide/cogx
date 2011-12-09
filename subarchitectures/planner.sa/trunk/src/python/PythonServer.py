import os, sys, traceback
from os import path

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

    def _log(self, level, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.info(msg)

    def trace(self, msg, *args):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
           args = args[0]
        if args:
           msg = msg % args
            
        self.log.trace(msg)

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
            
        #self.log.error(msg)

    def critical(self, msg, *args, **kwargs):
        self.connect()
        if args and (len(args) == 1) and args[0] and isinstance(args[0], dict):
            args = args[0]
        if args:
            msg = msg % args
            
        self.log.fatal(msg)

from standalone import config

config.set_logging_factory(CASTLoggerProxy)

#import de.dfki.lt.tr.beliefs.slice ## must be imported *before* Planner
from autogen import Planner
import cast.core
import cast.cdl

import standalone
from standalone import pddl

from standalone.planner import Planner as StandalonePlanner

from cast_task import CASTTask, TaskStateEnum
from display_client import PlannerDisplayClient

this_path = path.abspath(path.dirname(__file__))

def extend_pythonpath():
    """add standalone planner to PYTHONPATH"""
    standalone_path = path.join(this_path, "standalone")
    sys.path.insert(0, standalone_path)
    
extend_pythonpath()  

TEST_DOMAIN_FN = path.join(path.dirname(__file__), "domains/springtest.mapl")

log = config.logger()

def pdbdebug(fn):
    def decorated_method(self, *args, **kwargs):
        try:
            return fn(self, *args, **kwargs)
        except Exception, e:
            log.error("Python exception: %s", str(e))
            traceback.print_exception(*sys.exc_info())
            if self.start_pdb:
                import debug
                print "Entering debugger, please telnet to localhost:4444"
                debug.post_mortem()
            
    decorated_method.__name__ = fn.__name__
    decorated_method.__dict__ = fn.__dict__
    decorated_method.__doc__ = fn.__doc__
    return decorated_method

class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  
  def __init__(self):
    cast.core.CASTComponent.__init__(self)
    self.domain_fn = TEST_DOMAIN_FN
    self.problem_fn = None
    self.history_fn = None
    self.client = None
    self.dt = None
    self.hfc = None
    self.conceptual = None
    self.planner = None
    self.tasks = {}
    self.dt_tasks = {}
    self.expl_rules_fn = None

    self.beliefs = None
    self.address_dict = {}

    self.last_dt_id = 0
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

    if "--config" in config:
        standalone.globals.update_config(config.get("--config"))

    self.planner = StandalonePlanner()
    self.m_display.configureDisplayClient(config)
    
    self.client_name = config.get("--wm", "Planner")
    self.dt_name = config.get("--dt", "PlannerDTServer")
    self.coma_name = config.get("--coma", "hfcserver")
    self.conceptual_name = config.get("--conceptual", "conceptual.queryhandler")
    self.show_dot = "--nodot" not in config
    self.start_pdb = "--pdb" in config
    self.min_p = float(config.get("--low-p-threshold", 0.01))

    if "--dtdomain" in config:
        self.dtdomain_fn = path.join(standalone.globals.config.domain_dir, config["--dtdomain"])
    else:
        self.dtdomain_fn = None
        
    if "--dtproblem" in config:
        self.dtproblem_fn = path.join(standalone.globals.config.problem_dir, config["--dtproblem"])
    else:
        self.dtproblem_fn = None

    if "--default" in config:
        self.default_fn = path.join(standalone.globals.config.cogx_dir, config["--default"])
    else:
        self.default_fn = path.join(standalone.globals.config.cogx_dir, "instantiations/defaultprobs/defaultprobs.txt")
        
    if "--domain" in config:
      self.domain_fn = path.join(standalone.globals.config.domain_dir, config["--domain"])
      if not path.exists(self.domain_fn):
          log.error("Could not find specified domain %s. Using default domain %s", config["--domain"], TEST_DOMAIN_FN)
          self.domain_fn = TEST_DOMAIN_FN

    if "--expl_rules" in config:
      self.expl_rules_fn = path.join(standalone.globals.config.domain_dir, config["--expl_rules"])
      if not path.exists(self.expl_rules_fn):
          log.error("Could not find specified explanations rule set %s. Will not be able to determine explanations for failures!", config["--expl_rules"])
          self.expl_rules_fn = None

    if "--problem" in config:
      self.problem_fn = path.join(standalone.globals.config.problem_dir, config["--problem"])
      if not path.exists(self.problem_fn):
          self.problem_fn = path.join(standalone.globals.config.domain_dir, config["--problem"])
      if not path.exists(self.problem_fn):
          log.error("Could not find specified problem %s. Using planning state from CAST.", config["--problem"])
          self.problem_fn = None

    if "--history" in config:
      self.history_fn = path.join(standalone.globals.config.problem_dir, config["--history"])
      if not path.exists(self.history_fn):
          self.history_fn = path.join(standalone.globals.config.domain_dir, config["--history"])
      if not path.exists(self.history_fn):
          log.error("Could not find specified history %s. Using planning state from CAST.", config["--history"])
          self.history_fn = None
          
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

  def getHFC(self):
    import comadata
    if not self.hfc:
      self.hfc = self.getIceServer(self.coma_name, comadata.HFCInterface, comadata.HFCInterfacePrx)
      log.info("Connected to Comaserver %s", self.coma_name)
    return self.hfc

  def getConceptual(self):
    import ConceptualData
    if not self.conceptual:
      self.conceptual = self.getIceServer(self.conceptual_name, ConceptualData.QueryHandlerServerInterface, ConceptualData.QueryHandlerServerInterfacePrx)
      log.info("Connected to conceptual.sa query server %s", self.coma_name)
    return self.conceptual

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
      self.m_display.init_html()

  @pdbdebug
  def registerTask(self, task_desc, current=None):
    log.info("Planner PythonServer: New PlanningTask received:")
    standalone.globals.set_time()
    self.verbalise("Got a new task")

    task = CASTTask(task_desc, self.beliefs, self.domain_fn, self, problem_fn=self.problem_fn, expl_rules_fn=self.expl_rules_fn)
    # task = CASTTask(task_desc, self.beliefs, self.domain_fn, self, problem_fn=self.problem_fn, expl_rules_fn=self.expl_rules_fn)
    self.tasks[task.id] = task
    if task.status != Planner.Completion.FAILED:
        task.run()
    
  @pdbdebug
  def deliver_plan(self, task, slice_plan):
      task.status = Planner.Completion.SUCCEEDED
      self.m_display.update_task(task)
      self.getClient().deliverPlan(task.id, slice_plan, task.slice_goals);

  def get_new_attributed_beliefs(self, beliefs):
      import de.dfki.lt.tr.beliefs.slice as bm
      
      result = []
      for entry in beliefs:
          if isinstance(entry.belief.estatus, bm.epstatus.AttributedEpistemicStatus) \
                  and entry.belief.id not in self.address_dict:
              result.append(entry.belief)
      return result

  def process_beliefs(self, beliefs):
      result = []
      for entry in beliefs:
          result.append(entry.belief)
          self.address_dict[entry.belief.id] = entry.address
      return result

  @pdbdebug
  def updateState(self, state, percepts, current=None):
      log.debug("recieved state update.")
      new_attributed = self.get_new_attributed_beliefs(state)
      self.beliefs = self.process_beliefs(state)
      for task in self.tasks.itervalues():
          task.percepts += new_attributed
          task.percepts += self.process_beliefs(percepts)
          if task.internal_state == TaskStateEnum.WAITING_FOR_BELIEF:
              self.updateWaitingTask(task)

  @pdbdebug
  def updateWaitingTask(self, task):
      if not task.update_state(self.beliefs):
          log.warning("goal became invalid while waiting for action effects.")
          return

      standalone.globals.set_time()
      task.wait_update()

  @pdbdebug
  def taskTimedOut(self, task_desc, current=None):
      if task_desc.id not in self.tasks:
          log.warning("Warning: received update for task %d, but no such task found.", task_desc.id)
          return
      
      standalone.globals.set_time()
      task = self.tasks[task_desc.id]
      task.wait_timeout()
      
  @pdbdebug
  def updateTask(self, task_desc, current=None):
      if task_desc.id not in self.tasks:
          log.warning("Warning: received update for task %d, but no such task found.", task_desc.id)
          return
      
      standalone.globals.set_time()
      log.info("received task update")
      if task_desc.executionStatus in (Planner.Completion.SUCCEEDED, Planner.Completion.FAILED, Planner.Completion.ABORTED):
          log.info("task %d is done.", task_desc.id)
          self.m_display.remove_task(self.tasks[task_desc.id])
          del self.tasks[task_desc.id]
          return
        
      task = self.tasks[task_desc.id]

      if not task.update_state(self.beliefs):
          log.warning("Failed to update planning task. Setting status to FAILED.")
          task.update_status(TaskStateEnum.FAILED)
          self.deliver_plan(task, [])
          return

      if task.internal_state == TaskStateEnum.FAILED:
          log.info("Retry planning after failure")
          task.retry()
      elif task.dt_planning_active():
          task.action_executed_dt(task_desc.plan)
      else:
          task.action_executed_cp(task_desc.plan)


  @pdbdebug
  def notifyFailure(self, task_desc, current=None):
      if task_desc.id not in self.tasks:
          log.warning("Warning: received update for task %d, but no such task found.", task_desc.id)
          return
      
      standalone.globals.set_time()
      task = self.tasks[task_desc.id]
      task.handle_task_failure()
          
  @pdbdebug
  def start_dt_planning(self, task):
      assert task.dt_id is None, "There is already a DT task (%d) for task %d" % (task.dt_id, task.id)
      
      planning_tmp_dir = standalone.globals.config.tmp_dir
      task.dt_id = self.last_dt_id
      self.last_dt_id += 1
      
      tmp_dir = standalone.planner.get_planner_tempdir(planning_tmp_dir)
      domain_fn = path.join(tmp_dir, "domain%d.dtpddl" % task.dt_id)
      problem_fn = path.join(tmp_dir, "problem%d.dtpddl" % task.dt_id)
      log.info("Starting new DT task with id %d", task.dt_id)
      self.dt_tasks[task.dt_id] = task
      
      #Write dtpddl domain for debugging
      domain_out_fn = path.abspath(path.join(self.get_path(), "domain%d.dtpddl" % task.id))
      w = standalone.task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
      w.write(task.dt_task.problem, domain_fn=domain_out_fn)
      
      task.dt_task.write_dt_input(domain_fn, problem_fn, task.dt_id)
      self.getDT().newTask(task.dt_id, problem_fn, domain_fn);

  @pdbdebug
  def cancel_dt_session(self, task):
      assert task.dt_id is not None, "No DT task running for task %d" % task.id
      self.getDT().deliverObservation(task.dt_id, [])
      log.info("cancelled DT task %d for task %d", task.dt_id, task.id)
      task.dt_id = None

  @pdbdebug
  def queryGoal(self, beliefs, goal, current=None):
      import cast_state
      import task_preprocessor as tp
      
      standalone.globals.set_time()
      log.debug("Loading domain %s.", self.domain_fn)
      domain = pddl.load_domain(self.domain_fn)
      state = cast_state.CASTState([e.belief for e in beliefs], domain, component=self)
      cstate = state.state
      goalstrings = tp.transform_goal_string(goal, state.castname_to_obj).split("\n")
      try:
          pddl_goal = pddl.parser.Parser.parse_as(goalstrings, pddl.conditions.Condition, cstate.problem)
      except Exception, e:
          log.error("Could not parse goal: %s", goal)
          log.error("Error: %s", e.message)
          return False
      extstate = cstate.get_extended_state(cstate.get_relevant_vars(pddl_goal))
      return extstate.is_satisfied(pddl_goal)
      
  @pdbdebug
  def deliverAction(self, taskId, action, value, current=None):
      if taskId not in self.dt_tasks:
          log.warning("Warning: received action for DT task %d, but no such task found.", taskId)
          return
    
      log.info("%d: received new action from DT", taskId)
      log.debug("%d: expected value is %.2f", taskId, value)
      # if value < 50:
      #     log.info("%d: expected value is not good enough, requesting improvement.", taskId)
      #     self.getDT().improvePlanQuality(taskId)
      #     return
      
      task = self.dt_tasks[taskId]
      task.action_delivered(action)

      
  @pdbdebug
  def updateStatus(self, taskId, status, message, current=None):
      if taskId not in self.dt_tasks:
          log.warning("Warning: received state update for task %d, but no such task found.", taskId)
          return

      log.info("DT planner updates status to %s with the following message: %s", str(status), message)
      task = self.dt_tasks[taskId]

      #notify the cp planner that the subtask is finished
      if status == Planner.Completion.SUCCEEDED:
          task.dt_done()
          return
      
      #just forward the status for now
      self.getClient().updateStatus(task.id, status)

  
  def update_cast_beliefs(self, beliefs):
      temp_address = cast.cdl.WorkingMemoryAddress("temporary", "temporary")
      entries = [Planner.BeliefEntry(self.address_dict.get(b.id, temp_address), b) for b in beliefs]
      self.getClient().updateBeliefState(entries)

  def verbalise(self, phrase):
      self.getClient().verbalise(phrase)
      
