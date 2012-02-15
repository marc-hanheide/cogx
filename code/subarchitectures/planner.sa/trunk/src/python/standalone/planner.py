from __future__ import with_statement

import os, sys, shutil
import time
import re
import itertools as itools
from itertools import imap

import config, utils
import globals as global_vars

import task
from task import PlanningStatusEnum, Task
import pddl
import plans
import plan_postprocess
import statistics
import tempfile

log = config.logger("planner")

statistics_defaults = dict(
    planning_calls=0,
    planning_time=0.0,
    postprocess_time=0.0,
    translate_time=0.0,
    preprocess_time=0.0,
    search_time=0.0,
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
        if task._mapltask.goal == pddl.conditions.Falsity():
            task.set_plan(None, update_status=True)
            return

        replanning_necessary = self._evaluate_current_plan(task)
        if replanning_necessary:
            self._start_planner(task)
        else:
            if task.pending_action:
                # we're waiting for action affects to appear
                task.set_planning_status(PlanningStatusEnum.WAITING)
            #if no action is executing, trigger update
            elif not any(pnode.is_inprogress() for pnode in task.get_plan().nodes_iter()):
                log.info("no actions are executing, reissuing plan")
                task.set_plan(task.get_plan(), update_status=True)
            else:
                task.set_planning_status(PlanningStatusEnum.PLAN_AVAILABLE)
                

    def check_node(self, pnode, state, replan=False):
        conds = []
        if replan:
            read = pnode.replanconds
            universal = pnode.replan_universal
            conds.append(pnode.action.replan)
        else:
            read = pnode.preconds|pnode.replanconds
            universal = pnode.preconds_universal|pnode.replan_universal
            conds.append(pnode.action.precondition)
        for cnode in pnode.enabled_ceffs:
            # print "cconds:", map(str, cconds)
            read |= cnode.preconds
            universal |= cnode.preconds_universal
            conds.append(cnode.pddl_condition) #FIXME: proabably not correct yet

        negated_axioms = any(val == pddl.FALSE for svar, val in read if svar.function in state.problem.domain.derived or svar.modality in state.problem.domain.derived)
        # print map(str, state.problem.domain.derived)
        # print negated_axioms

        if not universal and not negated_axioms:
            #no universal preconditions => quickcheck
            if all(f in state for f in read):
                return True
            
        def has_preferences(cond, results):
            if isinstance(cond, pddl.conditions.PreferenceCondition):
                return True
            return any(results)
        
        if isinstance(pnode.action, plans.GoalAction) and conds and any(cond.visit(has_preferences) for cond in conds):
            if pnode.satisfied_softgoals:
                conds += list(pnode.satisfied_softgoals)

        action = pnode.action
        if conds:
            cond = pddl.Conjunction.join(conds)
            try:
                action.instantiate(pnode.full_args, state.problem)
                state.clear_axiom_cache()
                extstate = state.get_extended_state(state.get_relevant_vars(cond))
                result = extstate.is_satisfied(cond)
            except:
                return False
            finally:
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
                try:
                    pnode.action = mapltask.domain.get_action(pnode.action.name)
                except:
                    log.info("problem getting action description for %s", str(pnode))
                    return False
        return True

    def is_plan_valid(self, plan, goal_node, init_state):
        log.debug("checking plan validity.")
        #log.debug("current state is: %s", map(str, state.iterfacts()))
        #check for plan validity: test all preconditions and apply effects
        state = init_state.copy()

        skipped_actions = -1
        first_invalid_action = None
        for i,pnode in enumerate(plan):
            log.debug("Action: %s (%s)", str(pnode), str(pnode.status))
            if isinstance(pnode, plans.DummyNode) or pnode.status in (plans.ActionStatusEnum.EXECUTED, plans.ActionStatusEnum.FAILED) and not pnode.is_virtual():
                continue
            
            t1 = time.time()
            action = pnode.action
            #don't check preconditions of actions in progress
            if not pnode.is_inprogress() and (not self.check_node(pnode, state, replan=True) or not self.check_node(pnode, state)):
                if not first_invalid_action:
                    first_invalid_action = pnode
                log.debug("    not executable, trying to skip it.")
                # if an action is not executable, maybe it has already been executed
                # so we're trying if the rest of the plan is executable in the initial state
                skipped_actions = i
                state = init_state.copy()
            else:
                log.debug("    ok.")
                for f in pnode.effects:
                    state.set(f)
                for cnode in pnode.enabled_ceffs:
                    for f in cnode.effects:
                        state.set(f)
            log.trace("time for checking action (%s %s): %f", action.name, " ".join(a.name for a in pnode.full_args), time.time()-t1)

        t2 = time.time()
        #Now check if the goal is satisfied
        #log.debug("state after execution is: %s", map(str, state.iterfacts()))
        log.debug("checking if goal is still satisfied.")
        if self.check_node(goal_node, state):
            if skipped_actions > -1:
                log.info("Skipped the first %d actions.", skipped_actions)
                for pnode in plan[0:skipped_actions]:
                    pnode.status = plans.ActionStatusEnum.EXECUTED
                #we skipped all actions and the goal ist still satisfied: done
                if skipped_actions > len(plan)-2:
                    goal_node.status = plans.ActionStatusEnum.EXECUTED

            log.info("Plan is still valid.")
            log.debug("time for goal validation: %f", time.time()-t2)
            return True

        log.debug("time for goal validation: %f", time.time()-t2)
        
        if first_invalid_action:
            #print map(lambda (k,v): "%s = %s" % (str(k), str(v)), task.get_state().iteritems())
            log.info("Preconditions of (%s %s) are not satisfied, triggering replanning.", first_invalid_action.action.name, " ".join(a.name for a in first_invalid_action.full_args))
        else:
            log.info("Goal isn't fulfilled by the current plan, triggering replanning.")
        return False

    @statistics.time_method_for_statistics("monitoring_time")
    def _evaluate_current_plan(self, task):
        """
        Plan monitoring: checks whether a plan is still valid given
        a modified task description. 
        Returns True if replanning is necessary, else False.
        """

        task.pending_action = None
        
        if task.get_plan() is None:
            log.info("Plan is empty, replanning");
            return True

        self.statistics.increase_stat("monitoring_calls")

        if not self.update_plan(task.get_plan(), task._mapltask):
            log.info("Error updating plan, replanning.");
            return True
                
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
        if self.is_plan_valid(plan, task.get_plan().goal_node, task.get_state()):
            return False # No replanning neccessary

        #check if we're waiting for the last action:
        last_executed_index = -1
        for i, pnode in enumerate(plan):
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                last_executed_index = i
            elif pnode.status == plans.ActionStatusEnum.FAILED:
                last_executed_index = -1
                break
            
        if last_executed_index == -1:
            log.debug("total time for validation: %f", time.time()-t0)
            return True

        if task.wait_for_effects:
            #apply the action's effects and check if the plan would be executable
            pnode = plan[last_executed_index]
            remaining_plan = plan[last_executed_index+1:]
            for f in pnode.effects:
                state.set(f)
            for cnode in pnode.enabled_ceffs:
                for f in cnode.effects:
                    state.set(f)
            if self.is_plan_valid(remaining_plan, task.get_plan().goal_node, state):
                task.pending_action = pnode
                log.debug("total time for validation: %f", time.time()-t0)
                return False # waiting for action completion

        log.debug("total time for validation: %f", time.time()-t0)
        return True
        
    
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
        self.config = global_vars.config.base_planner.__dict__[self.__class__.__name__]
        self.executable = self.config.executable
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
        
        log.debug("\nNew plan is:")
#        log.debug("----------------------------------------------")
        for elem in output_data:
            log.debug(str(elem))
        log.debug("")

        with statistics.time_block_for_statistics(self.main_planner, "postprocess_time"):
            plan = self._post_process(output_data, task)
        return plan
    
    def _prepare_input(self, task):
        raise NotImplementedError
    
    def _run(self, task):
        raise NotImplementedError
    
    def _post_process(self, task):
        raise NotImplementedError

tmp_dir = None
    
def get_planner_tempdir(base_path):
    """creates a new subdirectory in base_path. If 'static_temp_dir' is set to True in
    config.ini, the subdirectory "static_dir_for_debugging" will be created (or cleared
    if it already exists). Otherwise, a random directory will be created in a race safe
    way (using the tmpfile module).
    """
    global tmp_dir
    if not tmp_dir:
        if global_vars.config.static_temp_dir:
            tmp_dir = os.path.join(base_path, "static_dir_for_debugging")
        else:
            tmp_dir = tempfile.mkdtemp(dir=base_path)
    if os.path.exists(tmp_dir):
        pass
        # utils.removeall(tmp_dir)  # remove old version
    else:
        os.makedirs(tmp_dir)
    log.debug("temp dir is: %s", tmp_dir)
    return tmp_dir

def clear_tmpdir(*args):
    global tmp_dir
    if not tmp_dir:
        return
    for file in args:
        fn = os.path.join(tmp_dir, file)
        if os.path.exists(fn):
            os.remove(fn)
    

class ContinualAxiomsFF(BasePlanner):
    """
    """
    PDDL_REXP = re.compile("\((.*)\)")
    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "plan.pddl", "stdout.out")]
        
        w = task.ADLOutput()
        w.write(_task.mapltask, domain_fn=paths[0], problem_fn=paths[1])
        
        # pddl_strs = _task.domain_str(task.PDDLWriter), _task.problem_str(task.PDDLWriter)
        # for path, content in zip(paths, pddl_strs):
        #     f = open(path, "w")
        #     f.write(content)
        #     f.close()
        return paths

    def _run(self, input_data, task):
        domain_path, problem_path, plan_path, stdout_path = input_data
        executable = os.path.join(global_vars.src_path, self.executable)
        cmd = "%(executable)s -o %(domain_path)s -f %(problem_path)s -O %(plan_path)s" % locals()
        proc,_,_ = utils.run_process(cmd, output=stdout_path, error=stdout_path)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, open(stdout_path).read(), log, "FF")
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
        clear_tmpdir("output", "output.sas", "sas_plan")

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "mutex.pddl", "output.sas", "output", "sas_plan", "stdout.out")]

        w = task.FDOutput()
        w.write(_task.mapltask, domain_fn=paths[0], problem_fn=paths[1], mutex_fn=paths[2])
        # dom_str = "\n".join(w.write_domain(_task.mapltask.domain, _task.mapltask))
        # prob_str = "\n".join(w.write_problem(_task.mapltask))
        # mutex_str = "\n".join(w.write_mutex(w.mutex_groups))
        # pddl_strs = dom_str, prob_str, mutex_str
        
        # for path, content in zip(paths, pddl_strs):
        #     f = open(path, "w")
        #     f.write(content)
        #     f.close()
        paths.append(tmp_dir)
        return paths
            

    def _run(self, input_data, task):
        import subprocess
        domain_path, problem_path, mutex_path, output_sas_path, output_path, plan_path, stdout_path, tmp_dir = input_data
        translate_path = os.path.join(global_vars.src_path, self.executable, self.config.translate)
        preprocess_path = os.path.join(global_vars.src_path, self.executable, self.config.preprocess)
        search_path = os.path.join(global_vars.src_path, self.executable, self.config.search)
        search_args = self.config.search_args

        output = open(stdout_path, "w")

        # print "sleep"
        # time.sleep(1)
        
        cmd = "%(translate_path)s  %(domain_path)s %(problem_path)s -m %(mutex_path)s" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "translate_time"):
            proc, translate_out,_ = utils.run_process(cmd, error=subprocess.STDOUT, dir=tmp_dir, wait=True)
        output.write(translate_out)
        log.debug("translate output:")
        log.debug(translate_out)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, translate_out, log, "Fast Downward Translate")
            return None

        cmd = "%(preprocess_path)s" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "preprocess_time"):
            proc, prep_out,_ = utils.run_process(cmd, error=subprocess.STDOUT, input=output_sas_path, dir=tmp_dir, wait=True)
        output.write(prep_out)
        log.debug("preprocess output:")
        log.debug(prep_out)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, prep_out, log, "Fast Downward Preprocess")
            return None

        cmd = "%(search_path)s %(search_args)s" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "search_time"):
             proc, search_out,_ = utils.run_process(cmd, error=subprocess.STDOUT, input=output_path, dir=tmp_dir, wait=True)
        output.write(search_out)
        log.debug("search output:")
        log.debug(search_out)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, search_out, log, "Fast Downward Search")
            return None

        output.close()
        
        try:
            pddl_output = open(plan_path).read()
        except IOError:
            log.info("Warning: Fast Downward did not find a plan.")
            log.info("Call was: %s", cmd)
            # log.warning("FD output was:\n\n>>>")
            # log.warning(translate_out)
            # log.warning(prep_out)
            # log.warning(search_out)
            # log.warning("<<<\n")
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
        
class ProbDownward(Downward):
    def _run(self, input_data, task):
        if task.deadline > -1:
            oldargs = self.config.search_args
            self.config.search_args += "d%d" % task.deadline
            res = Downward._run(self, input_data, task)
            self.config.search_args = oldargs
            return res
        else:
            return Downward._run(self, input_data, task)
    
class TFD(BasePlanner):
    """
    """
    TFD_REXP = re.compile("([0-9\.]*): \((.*)\) \[([0-9\.]*)\]")
    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)
        clear_tmpdir("output", "output.sas", "sas_plan")

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "output.sas", "output", "plan.best", "stdout.out")]
        w = task.TFDOutput()
        w.write(_task.mapltask, domain_fn=paths[0], problem_fn=paths[1])

        # pddl_strs = _task.domain_str(task.TFDWriter), _task.problem_str(task.TFDWriter)
        # for path, content in zip(paths, pddl_strs):
        #     f = open(path, "w")
        #     f.write(content)
        #     f.close()
        paths.append(tmp_dir)
        return paths

    def _run(self, input_data, task):
        import subprocess
        domain_path, problem_path, output_sas_path, output_path, plan_path, stdout_path, tmp_dir = input_data
        exec_path = os.path.join(global_vars.src_path, self.executable)
        search_args = self.config.search_args

        output = open(stdout_path, "w")
        
        cmd = "%(exec_path)s/translate/translate.py  %(domain_path)s %(problem_path)s" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "translate_time"):
            proc, _, translate_err = utils.run_process(cmd, output=output_sas_path, dir=tmp_dir, wait=True)
        
        output.write(translate_err)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, translate_err, log, "TFD Translate")
            return None

        cmd = "%(exec_path)s/preprocess/preprocess" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "preprocess_time"):
            proc, _, prep_err = utils.run_process(cmd, input=output_sas_path, output=output_path, dir=tmp_dir, wait=True)
        output.write(prep_err)
        
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, prep_err, log, "TFD Preprocess")
            return None

        cmd = "%(exec_path)s/search/search %(search_args)s" % locals()
        with statistics.time_block_for_statistics(self.main_planner, "search_time"):
             proc, _, search_err = utils.run_process(cmd, input=output_path, dir=tmp_dir, wait=True)
        output.write(search_err)

        if proc.returncode != 0:
            utils.print_errors(proc, cmd, search_err, log, "TFD Search")
            return None

        output.close()
        
        try:
            pddl_output = open(plan_path).read()
        except IOError:
            log.warning("Warning: TFD did not find a plan.")
            log.warning("Call was: %s", cmd)
            # log.warning("FD output was:\n\n>>>")
            # log.warning(translate_err)
            # log.warning(prep_err)
            # log.warning(search_err)
            # log.warning("<<<\n")
            return None
        
        pddl_plan = self.parse_tfd_output(pddl_output)
        return pddl_plan

    def parse_tfd_output(self, pddl_output):
        lines = [line for line in pddl_output.splitlines() if line]
        actions = []
        for line in lines:
            result = self.TFD_REXP.search(line)
            start = float(result.group(1))
            action =  result.group(2).lower()
            duration = float(result.group(3))
            #if start == 0:
            #    #start of a new (usually better plan)
            #    actions = []
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
        # plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())
        times_actions = [(a[0], a[1]) for a in action_list]
        times_actions = sorted(times_actions, key=lambda x:x[0])
        plan = plan_postprocess.make_po_plan(times_actions, task)
            
        return plan

class SAPA(BasePlanner):
    """
    """
    PLAN_REXP = re.compile("([0-9\.]*): \((.*)\) \[([0-9\.]*)\]")
    def __init__(self, main_planner):
        self.main_planner = main_planner
        self.config = global_vars.config.base_planner.__dict__[self.__class__.__name__]
        self.executable = self.config.executable
        # if not os.path.exists(self.executable):
        #     print "Executable %s does not exist!" % self.executable
        #     print "Please make sure it has been built and paths are set properly in config.ini."
        #     sys.exit(1)

    def _prepare_input(self, _task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = get_planner_tempdir(planning_tmp_dir)

        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "stdout.out")]
        
        w = task.ADLOutput()
        w.write(_task.mapltask, domain_fn=paths[0], problem_fn=paths[1])
        
        # pddl_strs = _task.domain_str(task.PDDLWriter), _task.problem_str(task.PDDLWriter)
        # for path, content in zip(paths, pddl_strs):
        #     f = open(path, "w")
        #     f.write(content)
        #     f.close()
        return paths

    def _run(self, input_data, task):
        import subprocess
        domain_path, problem_path, stdout_path = input_data
        java_class = self.executable
        classpath = os.path.join(global_vars.src_path, self.config.sapa_dir)
        cmd = "java -cp %(classpath)s %(java_class)s %(domain_path)s %(problem_path)s" % locals()
        output = open(stdout_path, "w")
        
        proc, planner_out,_ = utils.run_process(cmd, output=subprocess.PIPE, error=subprocess.PIPE)

        output.write(planner_out)
                
        if proc.returncode != 0:
            utils.print_errors(proc, cmd, open(planner_out).read(), log, "SAPA")
            return None
        
        pddl_plan = self.parse_sapa_output(planner_out)
        return pddl_plan

    def parse_sapa_output(self, pddl_output):
        lines = [line.strip() for line in pddl_output.splitlines() if line]

        def lines_from(lines, start, end):
            start_found = False if start is not None else True
            for l in lines:
                if end is not None and l == end:
                    return
                if start_found:
                    yield l
                elif l == start:
                    start_found = True

        actions = []
        for line in lines_from(lines, ";;-----------Original plan returned by Sapa-------------",
                               ";;-----------End original plan--------------------------"):
            result = self.PLAN_REXP.search(line)
            start = float(result.group(1))
            action =  result.group(2).lower()
            duration = float(result.group(3))
            #if start == 0:
            #    #start of a new (usually better plan)
            #    actions = []
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
        times_actions = [(start, a) for start, a, dur in action_list]
        times_actions = sorted(times_actions, key=lambda x:x[0])
        plan = plan_postprocess.make_po_plan(times_actions, task)

            
if __name__ == '__main__':    
    assert len(sys.argv) == 3, """Call 'planner.py domain.mapl task.mapl' for a single planner call"""
    domain_fn, problem_fn = sys.argv[1:]
    _task = Task()
    planner = Planner()
    _task.load_mapl_domain(domain_fn)
    _task.load_mapl_problem(problem_fn)
    planner._start_planner(_task)
    planner._evaluate_current_plan(_task)
    plan = _task.get_plan()
    ordered_plan = plan.topological_sort()
    for p in ordered_plan:
        print p

    G = plan.to_dot()
    dot_fn = "plan.dot"
    G.write(dot_fn)
    log.debug("Dot file for plan is stored in %s", dot_fn)
    
    log.info("Showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh")
    show_dot_script = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "show_dot.sh")
    os.system("%s %s" % (show_dot_script, dot_fn)) 
        
