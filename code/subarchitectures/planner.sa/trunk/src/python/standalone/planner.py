import os, sys, shutil
import time
import re
import itertools as itools
from itertools import imap

import config, utils
import globals as global_vars

import task
from task import PlanningStatusEnum, Task
import mapl_new as mapl
import plans
import plan_postprocess
import statistics
import tempfile

log = config.logger("planner")

statistics_defaults = dict(
    planning_calls=0,
    planning_time=0.0,
    monitoring_calls=0,
    monitoring_time=0.0,
    )


class Planner(object):
    """ 
    Base class for continual planners.  You can
    - register tasks
    - continually monitor and update the plan for this task automatically when the task is changed
    - call for emergency stops (if the underlying base planner supports this)
    """

    def __init__(self, plannerID=0, base_planner=None):
        self.plannerID = plannerID
        self.tasks = set()
        self.task_queue = [] # replace by real PQ as soon as scheduling policy becomes clearer
        if base_planner is None:
            base_planner_name = global_vars.config.base_planner.name
            base_planner = globals()[base_planner_name](self)
        self._base_planner = base_planner
        self._emergency_stop = False
        self.statistics = statistics.Statistics(defaults = statistics_defaults)

    __call_id_counter = 0

    @staticmethod
    def create_unique_planner_call_id(prefix=""):
        Planner.__call_id_counter += 1
        return prefix + str(Planner.__call_id_counter).zfill(3)

    def collect_statistics(self):
        """ return all stats collected by this planner """
        return self.statistics

    def register_task(self, task):
        if task not in self.tasks:
            self.task_queue.append(task)
            task.planner = self
        else:
            assert task.planner == self, "Task %s is already registered with another planner!" % task

    def emergency_stop(self):
        """
        """
        self._emergency_stop = True

    def stop_immediately(self):
        return self._emergency_stop
            
    def continual_planning(self, task):
        """
        Continual planning loop for an individual task.
        
        Arguments:
        - `task`: a planning task (usually from self.tasks)
        """
        if not task.is_dirty():
            return
        if task._mapltask.goal == mapl.conditions.Falsity():
            task.set_plan(None, update_status=True)
            return

        replanning_necessary = self._evaluate_current_plan(task)
        if replanning_necessary:
            self._start_planner(task)
        else:
            #if no action is executing, trigger update
            if not any(pnode.is_inprogress() for pnode in task.get_plan().nodes_iter()):
                log.info("no actions are executing, reissuing plan")
                task.set_plan(task.get_plan(), update_status=True)
            else:
                task.set_planning_status(PlanningStatusEnum.PLAN_AVAILABLE)
                

    def check_node(self, pnode, state, replan=False):
        if replan:
            read = pnode.replanconds
            universal = pnode.replan_universal
            cond = pnode.action.replan
        else:
            read = pnode.preconds|pnode.replanconds
            universal = pnode.preconds_universal|pnode.replan_universal
            if pnode.action.replan:
                cond = mapl.conditions.Conjunction([pnode.action.precondition, pnode.action.replan])
            else:
                cond = pnode.action.precondition

        if not universal:
            #no universal preconditions => quickcheck
            if all(f in state for f in read):
                return True

        action = pnode.action
        if cond:
            action.instantiate(pnode.full_args)
            extstate = state.get_extended_state(state.get_relevant_vars(cond))
            result = extstate.is_satisfied(cond)
            action.uninstantiate()
            return result
        
        return True

    def update_plan(self, plan, mapltask):
        for pnode in plan.nodes_iter():
            if isinstance(pnode.action, plans.GoalAction):
                pnode.action.precondition = mapltask.goal
            elif isinstance(pnode.action, plans.DummyAction):
                continue
            else:
                pnode.action = mapltask.get_action(pnode.action.name)

    @statistics.time_method_for_statistics("monitoring_time")
    def _evaluate_current_plan(self, task):
        """
        Plan monitoring: checks whether a plan is still valid given
        a modified task description. 
        Returns True if replanning is necessary, else False.
        """
        if task.get_plan() is None:
            return True

        self.statistics.increase_stat("monitoring_calls")

        self.update_plan(task.get_plan(), task._mapltask)
                
        t0 = time.time()
        state = task.get_state().copy()
        plan = task.get_plan().topological_sort()

        #check if the goal is already satisfied
        if self.check_node(task.get_plan().goal_node, state):
            log.info("Goal is reached")
            for pnode in plan:
                pnode.status = plans.ActionStatusEnum.EXECUTED
            return False

        #check for expandable assertions
        for pnode in plan:
            if isinstance(pnode, plans.DummyNode):
                continue

            if pnode.action.replan:
                if self.check_node(pnode, state, replan=True):
                    log.info("Assertion (%s %s) is expandable, triggering replanning.", pnode.action.name, " ".join(a.name for a in pnode.full_args))
                    task.statistics.increase_stat("deliberate_replans")
                    return True
            
        log.debug("time for checking assertions: %f", time.time()-t0)

        log.debug("checking plan validity.")
        #log.debug("current state is: %s", map(str, state.iterfacts()))
        #check for plan validity: test all preconditions and apply effects
        skipped_actions = -1
        first_invalid_action = None
        for i,pnode in enumerate(plan):
            if isinstance(pnode, plans.DummyNode) or pnode.status in (plans.ActionStatusEnum.EXECUTED, plans.ActionStatusEnum.FAILED):
                continue
            
            t1 = time.time()
            action = pnode.action
            #don't check preconditions of actions in progress
            if not pnode.is_inprogress() and (not self.check_node(pnode, state, replan=True) or not self.check_node(pnode, state)):
                if not first_invalid_action:
                    first_invalid_action = pnode
                log.debug("Action (%s %s) is not executable, trying to skip it.", action.name, " ".join(a.name for a in pnode.full_args))
                # if an action is not executable, maybe it has already been executed
                # so we're trying if the rest of the plan is executable in the initial state
                skipped_actions = i
                state = task.get_state().copy()
            else:
                log.debug("Action (%s %s) ok.", action.name, " ".join(a.name for a in pnode.full_args))
                for f in pnode.effects:
                    state.set(f)
            log.debug("time for checking action (%s %s): %f", action.name, " ".join(a.name for a in pnode.full_args), time.time()-t1)

        t2 = time.time()
        #Now check if the goal is satisfied
        #log.debug("state after execution is: %s", map(str, state.iterfacts()))
        log.debug("checking if goal is still satisfied.")
        if self.check_node(task.get_plan().goal_node, state):
            if skipped_actions > -1:
                log.info("Skipped the first %d actions.", skipped_actions)
                for pnode in plan[0:skipped_actions]:
                   pnode.status = plans.ActionStatusEnum.EXECUTED
                #we skipped all actions and the goal ist still satisfied: done
                if skipped_actions > len(plan)-2:
                    task.get_plan().goal_node.status = plans.ActionStatusEnum.EXECUTED

            log.info("Plan is still valid.")
            log.debug("time for goal validation: %f", time.time()-t2)
            log.debug("total time for validation: %f", time.time()-t0)
            return False

        log.debug("time for goal validation: %f", time.time()-t2)
        log.debug("total time for validation: %f", time.time()-t0)
        
        if first_invalid_action:
            #print map(lambda (k,v): "%s = %s" % (str(k), str(v)), task.get_state().iteritems())
            log.info("Preconditions of (%s %s) are not satisfied, triggering replanning.", first_invalid_action.action.name, " ".join(a.name for a in first_invalid_action.full_args))
        else:
            log.info("Goal isn't fulfilled by the current plan, triggering replanning.")

        return True  # no monitoring currently

    
    @statistics.time_method_for_statistics("planning_time")
    def _start_planner(self, task):
        """
        Call base planner for this task.  The base planner should
        run in a separate thread or process to that this function
        can return immediately.  
        """
        # TODO: currently atomic process, ie planner will wait for result
        log.info("Planning was triggered for task %d.", task.taskID)
        self.statistics.increase_stat("planning_calls")
        plan = self._base_planner.find_plan(task)
        task.set_plan(plan)
        
        
class BasePlanner(object):
    """
    """
    def __init__(self, main_planner):
        self.main_planner = main_planner
        self.executable = global_vars.config.base_planner.__dict__[self.__class__.__name__].executable
        if not os.path.exists(self.executable):
            print "Executable %s does not exist!" % self.executable
            print "Please make sure it has been built and paths are set properly in config.ini."
            sys.exit(1)

    def find_plan(self, task):
        """
        """
        input_data = self._prepare_input(task)
        output_data = self._run(input_data, task)
        if output_data is None:
            return None
        plan = self._post_process(output_data, task)
        return plan
    
    def _prepare_input(self, task):
        raise NotImplementedError
    
    def _run(self, task):
        raise NotImplementedError
    
    def _post_process(self, task):
        raise NotImplementedError

def get_planner_tempdir(base_path):
    """creates a new subdirectory in base_path. If 'static_temp_dir' is set to True in
    config.ini, the subdirectory "static_dir_for_debugging" will be created (or cleared
    if it already exists). Otherwise, a random directory will be created in a race safe
    way (using the tmpfile module).
    """
    if global_vars.config.static_temp_dir.lower() == "true":
        tmp_dir = os.path.join(base_path, "static_dir_for_debugging")
        if os.path.exists(tmp_dir):
            utils.removeall(tmp_dir)  # remove old version
        return tmp_dir
    return tempfile.mkdtemp(dir=base_path)

class ContinualAxiomsFF(BasePlanner):
    """
    """
    PDDL_REXP = re.compile("\((.*)\)")
    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)
        
        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "plan.pddl", "stdout.out")]
        pddl_strs = _task.domain_str(task.PDDLWriter), _task.problem_str(task.PDDLWriter)
        for path, content in zip(paths, pddl_strs):
            f = open(path, "w")
            f.write(content)
            f.close()
        return paths

    def _run(self, input_data, task):
        domain_path, problem_path, plan_path, stdout_path = input_data
        executable = os.path.join(global_vars.src_path, self.executable)
        cmd = "%(executable)s -o %(domain_path)s -f %(problem_path)s -O %(plan_path)s" % locals()
        proc = utils.run_process(cmd, output=stdout_path, error=stdout_path)
        
#        stdout_output = utils.run_command(cmd, output=stdout_path)
#         print "Planner output:"
#         print stdout_output
        
        if proc.returncode != 0:
            print "Warning: FF returned with nonzero exitcode:\n\n>>>"
            print "Call was:", cmd
            print open(stdout_path).read()
            print "<<<\n"
            if proc.returncode > 0:
                print "Exit code was %d" % proc.returncode
            else:
                print "Killed by signal %d" % -proc.returncode
            return None

        try:
            pddl_output = open(plan_path).read()
        except IOError:
            print "Warning: FF did not find a plan or crashed."
            print "Call was:", cmd
            print "FF output was:\n\n>>>"
            print open(stdout_path).read()
            print "<<<\n"
            return None
        pddl_plan = self.parse_ff_output(pddl_output)
        return pddl_plan

    def parse_ff_output(self, pddl_output):
        lines = [line for line in pddl_output.splitlines() if line]
        result = lines.pop(0)
        assert result in ("SUCCESS", "FAILURE"), "wrong format for FF output"
        if result == "FAILURE":
            return None
        actions = [self.PDDL_REXP.search(line).group(1).lower() for line in lines]
        return actions

    def _post_process(self, action_list, task):
        """
        Receives a PDDL action list and produces a MAPL plan
        Steps:
        - create an empty MAPL plan P
        - add dummy action for the current state of the task to P
        - create goal action
        - map PDDL actions to properly instantiated MAPL actions
        - determine causal and threat-prevention links between actions
        - add actions and links to P
        - add goal action to P
        TODO:
        - derived predicates: where are they used in preconds and where have they been triggered
        - knowledge preconditions: make them explicit?
        - negotiation actions --> requests: when and how?
        """
        # very preliminary implementation of the above!
        plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())
        times_actions = enumerate(action_list)  # keep it sequentially for now
        plan = plan_postprocess.make_po_plan(times_actions, task)
        return plan

            
class Downward(BasePlanner):
    """
    """
    PDDL_REXP = re.compile("\((.*)\)")
    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "mutex.pddl", "sas_plan", "stdout.out")]

        w = task.FDWriter()
        dom_str = "\n".join(w.write_domain(_task.mapltask.domain))
        prob_str = "\n".join(w.write_problem(_task.mapltask))
        mutex_str = "\n".join(w.write_mutex(w.mutex_groups))
        pddl_strs = dom_str, prob_str, mutex_str
        
        for path, content in zip(paths, pddl_strs):
            f = open(path, "w")
            f.write(content)
            f.close()
        paths.append(tmp_dir)
        return paths

    def _run(self, input_data, task):
        domain_path, problem_path, mutex_path, plan_path, stdout_path, tmp_dir = input_data
        executable = os.path.join(global_vars.src_path, self.executable)
        cmd = "%(executable)s  %(domain_path)s %(problem_path)s %(mutex_path)s" % locals()
        proc = utils.run_process(cmd, output=stdout_path, error=stdout_path, dir=tmp_dir)
        
#        stdout_output = utils.run_command(cmd, output=stdout_path)
#         print "Planner output:"
#         print stdout_output
        
        if proc.returncode != 0:
            print "Warning: Fast Downward returned with nonzero exitcode:\n\n>>>"
            print "Call was:", cmd
            print open(stdout_path).read()
            print "<<<\n"
            if proc.returncode > 0:
                print "Exit code was %d" % proc.returncode
            else:
                print "Killed by signal %d" % -proc.returncode
            return None

        try:
            pddl_output = open(plan_path).read()
        except IOError:
            print "Warning: Fast Downward did not find a plan or crashed."
            print "Call was:", cmd
            print "FD output was:\n\n>>>"
            print open(stdout_path).read()
            print "<<<\n"
            return None
        pddl_plan = self.parse_fd_output(pddl_output)
        return pddl_plan

    def parse_fd_output(self, pddl_output):
        lines = [line for line in pddl_output.splitlines() if line]
        actions = []
        for line in lines:
            result = self.PDDL_REXP.search(line)
            action =  result.group(1).lower()
            actions.append(action)
                
        return actions

    def _post_process(self, action_list, task):
        """
        Receives a PDDL action list and produces a MAPL plan
        Steps:
        - create an empty MAPL plan P
        - add dummy action for the current state of the task to P
        - create goal action
        - map PDDL actions to properly instantiated MAPL actions
        - determine causal and threat-prevention links between actions
        - add actions and links to P
        - add goal action to P
        TODO:
        - derived predicates: where are they used in preconds and where have they been triggered
        - knowledge preconditions: make them explicit?
        - negotiation actions --> requests: when and how?
        """
        plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())
        times_actions = enumerate(action_list)  # keep it sequentially for now
        plan = plan_postprocess.make_po_plan(times_actions, task)
        return plan
        

    
class TFD(BasePlanner):
    """
    """
    TFD_REXP = re.compile("([0-9\.]*): \((.*)\) \[([0-9\.]*)\]")
    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "stdout.out")]
        pddl_strs = _task.domain_str(task.TFDWriter), _task.problem_str(task.TFDWriter)
        for path, content in zip(paths, pddl_strs):
            f = open(path, "w")
            f.write(content)
            f.close()
        paths.append(tmp_dir)
        return paths

    def _run(self, input_data, task):
        domain_path, problem_path, stdout_path, tmp_dir = input_data
        translate = os.path.join(self.executable, "translate/translate.py")
        preprocess = os.path.join(self.executable, "preprocess/preprocess")
        search = os.path.join(self.executable, "search/search")
        print "running..."
        cmd = "%(translate)s %(domain_path)s %(problem_path)s" % locals()
        p_tr = utils.run_process(cmd, dir=tmp_dir)
        if p_tr.returncode != 0:
            print "Warning: Error in TFD translate. output was:\n\n>>>"
            print p_tr.stderr.read()
            print "<<<\n"
            return None
        print "translate done"
        p_pre = utils.run_process(preprocess, input=p_tr.stdout, dir=tmp_dir)
        if p_pre.returncode != 0:
            print "Warning: Error in TFD preprocess. output was:\n\n>>>"
            print p_pre.stderr.read()
            print "<<<\n"
            return None
        
        print "preprocess done"
        cmd = "%(search)s yY t 5 -" % locals()
        p_search = utils.run_process(cmd, input=p_pre.stdout, dir=tmp_dir)
        
        if p_search.returncode != 0:
            print "Warning: TFD search did not find a plan or crashed.  TFD output was:\n\n>>>"
            print p_search.stderr.read()
            print "<<<\n"
            return None

        print "all done"
        output = open(stdout_path, "w")
        output.write(p_tr.stderr.read())
        output.write(p_pre.stderr.read())
        output.write(p_search.stderr.read())
        output.close()

        pddl_plan = self.parse_tfd_output(p_search.stdout.read())
        return pddl_plan

    def parse_tfd_output(self, pddl_output):
        lines = [line for line in pddl_output.splitlines() if line]
        actions = []
        for line in lines:
            result = self.TFD_REXP.search(line)
            start = float(result.group(1))
            action =  result.group(2).lower()
            duration = float(result.group(3))
            if start == 0:
                #start of a new (usually better plan)
                actions = []
            actions.append((start, action, duration))
                
        return actions

    def _post_process(self, action_list, task):
        """
        Receives a PDDL action list and produces a MAPL plan
        Steps:
        - create an empty MAPL plan P
        - add dummy action for the current state of the task to P
        - create goal action
        - map PDDL actions to properly instantiated MAPL actions
        - determine causal and threat-prevention links between actions
        - add actions and links to P
        - add goal action to P
        TODO:
        - derived predicates: where are they used in preconds and where have they been triggered
        - knowledge preconditions: make them explicit?
        - negotiation actions --> requests: when and how?
        """
        plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())
        times_actions = [(a[0], a[1]) for a in action_list]
        plan = plan_postprocess.make_po_plan(times_actions, task)
            
        return plan
    
            
if __name__ == '__main__':    
    assert len(sys.argv) == 3, """Call 'planner.py domain.mapl task.mapl' for a single planner call"""
    domain_fn, problem_fn = sys.argv[1:]
    task = Task()
    planner = Planner()
    task.load_mapl_domain(domain_fn)
    task.load_mapl_problem(problem_fn)
    planner._start_planner(task)
    planner._evaluate_current_plan(task)
    plan = task.get_plan()
