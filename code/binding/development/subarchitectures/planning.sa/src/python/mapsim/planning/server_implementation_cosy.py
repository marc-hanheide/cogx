#!/usr/bin/env python

"""The planning server for CAST.

This can also be used for standalone testing as follows:

usage: %prog [options] [problem domain agent]
  -a, --ack : turn on/off producing acks for new tasks [default: %default]
  -d, --debug : show all log messages [default: %default]
  -v, --verbosity=VERBOSITY : general verbosity level [default: %default]
  -i, --info=INFO : info on specific parts of MAPSIM [default: %default]
"""

import os, os.path, sys
from os.path import dirname, abspath
import random
from collections import defaultdict

# make sure both the mapsim dir and the planning dir are in the path
PLANNING_DIR = dirname(abspath(__file__))
MAPSIM_DIR = dirname(PLANNING_DIR)  # one below
for path in [MAPSIM_DIR, PLANNING_DIR]:
    if path not in sys.path:
        sys.path[1:1] = [path]


import config
from config import log
from constants import *
import agent
import state 
import mapl
import mapsim_planner
import idl_classes   # extended IDL stubs
from idl_classes import GOAL_ACHIEVABLE, GOAL_UNACHIEVABLE, GOAL_ACHIEVED, OLD_PLAN_KEPT, CHANGED_PLAN

COSY_CONFIG = dict(
    #info = INFO_PARTIAL_ORDER,
    verbosity = 1,
    debug=False,
    ack=False
)

PRODUCE_ACK_FIRST = True

class Planner(object):
    def __init__(self):
        self._task_id_counter = 0
        self.tasks = {}
        self.accepted_tasks = set()  # IDs of accepted tasks
        self.caches = defaultdict(dict)
        self.execution_history = defaultdict(list)

    def new_task(self, cast_task):
        """ Register a new task with the planner.  Side effect: sets a
        unique TaskID for that task """
        try:
            log("Planner entering new_task()", vlevel=1)
            idl_classes.convert2extended(cast_task)
            self._task_id_counter += 1
            cast_task.task_id = task_id = "task_%d" % self._task_id_counter
            mapl_task = cast_task.to_MAPL()
            planning_agent = agent.Agent(name=cast_task.planning_agent)
            mapl_task.planning_agent = planning_agent  
            mapl_task.planning_agent.problem = mapl_task # cyclic... horrible! change ASAP!!!
            import main
            data_dir = os.path.join(config.common_data_path, "cast_planner")
            main.create_and_link_to_data_dir(data_dir)
            config.setup_log_and_stats("cast_scenario")
            planning_agent.domain_data = mapl.prepare_compiled_planning_domains(cast_task.domain_fn, planning_agent)
            config.mapl_domain = planning_agent.domain_data.mapl_domain
            planning_agent.adopt_mapl_task(mapl_task)
            config.planner = mapsim_planner.MapsimPlanner()
            # check acceptability here in the future (currently, only in continual_planning)
            self.tasks[task_id] = mapl_task
            log("Planner leaving new_task()", vlevel=1)
            return cast_task
        except Exception, e:
            log(e, vlevel=1)
            import traceback
            traceback.print_exc(file=sys.stdout)
            raise

    def load_mapl_task(self, task_fn, mapl_dom_path, planning_agent):
        mapl_domain = config.mapl_domain = mapl.load_mapl_domain(mapl_dom_path)
        problem = mapl.load_mapl_task(task_fn, mapl_domain)
        problem.planning_agent = planning_agent
        cast_task = idl_classes.PlanningTask.from_MAPL(problem)
        cast_task.domain_fn = mapl_dom_path
        cast_task.planning_agent = planning_agent
        return cast_task

    def current_task_state(self, task_id):
        mapl_task = self.get_mapl_task(task_id)
        mapl_task = mapl_task.planning_agent.problem
        cast_task = idl_classes.PlanningTask.from_MAPL(mapl_task, task_id)
        cast_task.task_id = task_id
        idl_classes.convert2stub(cast_task)
        return cast_task

    def change_task(self, cast_task, clear_first):
        if not clear_first:
            raise NotImplementedError
        task_id = cast_task.task_id
        old_task = self.get_mapl_task(task_id)
        idl_classes.convert2extended(cast_task)
        mapl_task = cast_task.to_MAPL()
        mapl_task.planning_agent = old_task.planning_agent
        mapl_task.planning_agent.update_task(mapl_task )
        self.tasks[task_id] = mapl_task

    def continual_planning(self, task_id):
        mapl_task = self.get_mapl_task(task_id)
        mapl_task.ccp_state = mapl_task.planning_agent.monitor_and_replan()
        return mapl_task.ccp_state

    def next_executable_plan_steps(self, task_id):
        mapl_task = self.get_mapl_task(task_id)
        planning_agent = mapl_task.planning_agent
        ccp_state = mapl_task.ccp_state
        if task_id not in self.accepted_tasks and PRODUCE_ACK_FIRST:
            self.accepted_tasks.add(task_id)
            candidates = [self.produce_ack_for_ccp_state(task_id)]
        else:
            plan = ccp_state.plan
            if DEBUG: log(plan, vlevel=1)
            #initial_candidates = list(planning_agent.get_candidate_commands(plan))
            initial_candidates = planning_agent.compute_next_commands()
            initial_candidates = self.postprocess_next_steps(initial_candidates)
            candidates = [idl_classes.Command.from_MAPL(cmd, task_id) for cmd in initial_candidates]
            for cmd, new_cmd in zip(initial_candidates, candidates):
                self.cache(task_id, new_cmd, cmd)
        log("Next executable steps: %s" % [str(s) for s in candidates], vlevel=1)
        return candidates

    def expected_changes(self, task_id, cmd):
        idl_classes.convert2extended(cmd)
        mapl_task = self.get_mapl_task(task_id)
        planning_agent = mapl_task.planning_agent
        try:
            mapl_cmd = self.cached_item(task_id, cmd)
            mapl_changes = mapl_cmd.mapl_action.rw_description.written_facts
        except (KeyError,AttributeError):
            # if a command is not cached or is not a maplaction, then just
            # ignore it (for the time being)
            return []
        convert_fct = idl_classes.Fact.from_MAPL 
        facts_idl = [convert_fct(svar, val) for svar, val in mapl_changes.items()]
        return facts_idl

    def command_was_executed(self, task_id, cmd, success):
        idl_classes.convert2extended(cmd)
        mapl_task = self.get_mapl_task(task_id)
        planning_agent = mapl_task.planning_agent
        if success:
            self.execution_history[task_id].append(cmd)
            hist_strs = [str(cmd) for cmd in self.execution_history[task_id]] 
            log("execution history so far: %s" % hist_strs, vlevel=1)
        else:
            log("execution of the following command failed: %s" % cmd, vlevel=1)
        try:
            mapl_cmd = self.cached_item(task_id, cmd)
            planning_agent.react_to_execution(mapl_cmd, success)
        except KeyError:
            # if a command is not cached, then just ignore it (for the time being)
            pass

    def negative_sensing(self, task_id, sensing_action):
        raise NotImplementedError

    def self_test(self):
        log("Planner self test running...", vlevel=1)
        task = prepare_test_task(p)
        run_test(self, task)

    #### helpers ####

    def get_mapl_task(self, task_id):
        try:
            return self.tasks[task_id]
        except KeyError:
            log("Error: Task ID '%s' unknown." % task_id, vlevel=1)
            raise

    def current_ccp_state(self, task_id):
        mapl_task = self.get_mapl_task(task_id)
        ccp_state = mapl_task.ccp_state
        return ccp_state

    def current_plan(self, task_id):
        return self.current_ccp_state(task_id).plan

    def cache(self, task_id, item1, item2, bidirectional=True):
        cache = self.caches[task_id]
        cache[item1] = item2
        if bidirectional:
            cache[item2] = item1
            
    def cached_item(self, task_id, item):
        cache = self.caches[task_id]
        try:
            return cache[item]
        except KeyError:
            log("-----", vlevel=1)
            for key in cache:
                log("seeking: %s" % item, vlevel=1)
                log("cached: %s " % key, vlevel=1)
                log("-----", vlevel=1)
            raise KeyError("Could not find cached item '%s'" % item)

    def update_state(self, task_id, changes):
        # this (or something similar) should probably go in the IDL for convenience
        mapl_task = self.get_mapl_task(task_id)
        state = mapl_task.init_state
        mapl_changes = [fact.to_MAPL() for fact in changes]
        state.update(mapl_changes)
        return self.current_task_state(task_id)

    def produce_ack_for_ccp_state(self, task_id):
        ccp_state = self.current_ccp_state(task_id)
        ack_cls = {GOAL_UNACHIEVABLE : idl_classes.AckGoalRejected, 
                   GOAL_ACHIEVED : idl_classes.AckGoalHoldsAlready, 
                   GOAL_ACHIEVABLE : idl_classes.AckGoalAccepted}           
        return ack_cls[ccp_state.execution_state](task_id)

    def postprocess_next_steps(self, candidates):
        """ do CAST-specific stuff with the list of possible next actions.
        For example, we may want to combine several speech acts into
        one."""
        return candidates   # nothing yet


########################

def prepare_test_task(planner):
    # prepare test data
    dom_name = "coffee"
    dom_path = os.path.join(MAPSIM_DIR, "domains", dom_name)
    mapl_dom_path = os.path.join(dom_path, "mapl_files/domain_agents.mapl")
    scenario_path = "one_agent/prob001/"
    agent_name = "R2D2"
    module_path = os.path.join(dom_path, "module.py")
    task_fn = os.path.join(dom_path, "scenarios", scenario_path, "%s.mapl" % agent_name)
    initial_task = planner.load_mapl_task(task_fn, mapl_dom_path, agent_name)
    return initial_task

def plan_once(planner, current_task):
    assert isinstance(current_task, idl_classes.PlanningTask)
    planner.new_task(current_task)
    if DEBUG: log("The following test task will be used:\n%s" % current_task, vlevel=1)
    task_id = current_task.task_id
    if DEBUG: log("Current task:\n%s" % current_task, vlevel=1)
    # monitor the plan and eventually repair it
    ccp_state = planner.continual_planning(task_id)
    if DEBUG and ccp_state.planning_state == idl_classes.CHANGED_PLAN:
        log("Replanning was triggered.", vlevel=1)
    if ccp_state.execution_state == idl_classes.GOAL_UNACHIEVABLE:
        # no way to achieve the goal anymore --> failure
        log("Test failed.  Could not achieve goal.", vlevel=1)
        sys.exit()
    log("Plan found:\n%s" % planner.current_plan(task_id), vlevel=1)
    # get executable steps from the current plan
    possible_commands = planner.next_executable_plan_steps(task_id)
    log("Possible commands: %s" % [str(cmd) for cmd in possible_commands], vlevel=1)


def run_test(planner, current_task):
    assert isinstance(current_task, idl_classes.PlanningTask)
    # the following test process should be easily reproducible on the CAST/Java level
    # "planner" is the CORBA server to call (with the same methods used here)
    planner.new_task(current_task)
    if DEBUG: log("The following test task will be used:\%s" % current_task, vlevel=1)
    task_id = current_task.task_id
    cmd_history = []
    while True:
        if DEBUG: log("Current task:\n%s" % current_task, vlevel=1)
        # monitor the plan and eventually repair it
        ccp_state = planner.continual_planning(task_id)
        if ccp_state.planning_state == idl_classes.CHANGED_PLAN:
            log("Replanning was triggered.", vlevel=1)
        if ccp_state.execution_state == idl_classes.GOAL_ACHIEVED:
            # goal achieved
            break
        elif ccp_state.execution_state == idl_classes.GOAL_UNACHIEVABLE:
            # no way to achieve the goal anymore --> failure
            log("Test failed.  Could not achieve goal.", vlevel=1)
            sys.exit()
        # get executable steps from the current plan
        possible_commands = planner.next_executable_plan_steps(task_id)
        if DEBUG: log("Possible commands: %s" % [str(cmd) for cmd in possible_commands], vlevel=1)
        command = random.choice(possible_commands)
        if DEBUG: log("Chosen command: %s" % command, vlevel=1)
        changes = planner.expected_changes(task_id, command)
        if DEBUG: log("Expected changes: %s" % changes, vlevel=1)
        # for now simply assume successful execuction
        # inform planner about execuction
        log("Simulating execuction of %s" % command, vlevel=1)
        planner.command_was_executed(task_id, command, success=True)
        cmd_history.append(command)
        # assume perfect execution and full observability here:
        # add expected_changes as real changes!
        #planner.update_state(task_id, changes)
        # not necessary here because command_was_executed() does it
        # already. do we want that?
        current_task = planner.current_task_state(task_id)
    log("problem solved by executing the following commands:", vlevel=1)
    log([str(cmd) for cmd in cmd_history], vlevel=1)


def parse_command_line():
    # parse options and required arguments from docstring automatically
    from myoptionparse import OptionParser
    parser = OptionParser(from_doc=__doc__)
    parser.set_defaults_from(COSY_CONFIG)
    options, args = parser.parse_args()
    options.additional_args = " ".join(args)
    config.__dict__.update(options.__dict__)
    return options

if __name__ == "__main__":
    options = parse_command_line()
    DEBUG = options.debug
    idl_classes.DEBUG = DEBUG
    if options.ack: 
        PRODUCE_ACK_FIRST = not PRODUCE_ACK_FIRST
    config.verbosity = options.verbosity
    p = Planner()
    if options.problem:
        if not options.domain or not options.agent:
            log("Direct planner call needs a problem file, a domain file and an agent name!", vlevel=1)
            log("Example: ./mapl_planner prob.mapl domain.mapl R2D2", vlevel=1)
            log("Call with --help for more information.", vlevel=1)
            sys.exit()
        task = p.load_mapl_task(options.problem, options.domain, options.agent)
        plan_once(p, task)
    else:
        p.self_test()
