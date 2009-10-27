import os, sys, shutil
import time
import re
import itertools as itools
from itertools import imap

import utils
import globals as global_vars

from task import PlanningStatusEnum, Task
import mapl_new as mapl
import plans
import plan_postprocess

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

    __call_id_counter = 0

    @staticmethod
    def create_unique_planner_call_id(prefix=""):
        Planner.__call_id_counter += 1
        return prefix + str(Planner.__call_id_counter).zfill(3)

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
            if not any(map(lambda pnode: pnode.is_inprogress(), task.get_plan().V)):
                print "no actions are executing, reissuing plan"
                task.set_plan(task.get_plan(), update_status=True)
            else:
                task.set_planning_status(PlanningStatusEnum.PLAN_AVAILABLE)
                

    def check_node(self, pnode, state, replan=False):
        if replan:
            read = pnode.replanconds
            universal = pnode.replan_universal
            cond = pnode.action.replan
        else:
            read = pnode.preconds
            universal = pnode.preconds_universal
            cond = pnode.action.precondition

        if not universal:
            #no universal preconditions => quickcheck
            return all(imap(lambda f: f in state, read))
        
        action = pnode.action
        if cond:
            action.instantiate(pnode.full_args)
            extstate = state.getExtendedState(state.getRelevantVars(cond))
            result = extstate.isSatisfied(cond)
            action.uninstantiate()
            return result
        
        return True

    def update_plan(self, plan, mapltask):
        for pnode in plan.V:
            if isinstance(pnode.action, plans.GoalAction):
                pnode.action.precondition = mapltask.goal
            elif isinstance(pnode.action, plans.DummyAction):
                continue
            else:
                pnode.action = mapltask.getAction(pnode.action.name)
    
    def _evaluate_current_plan(self, task):
        """
        Plan monitoring: checks whether a plan is still valid given
        a modified task description. 
        Returns True if replanning is necessary, else False.
        """
        if task.get_plan() is None:
            return True

        self.update_plan(task.get_plan(), task._mapltask)
                
        t0 = time.time()
        state = task.get_state().copy()
        plan = task.get_plan().topological_sort(include_depths=False)

        #check if the goal is already satisfied
        if self.check_node(task.get_plan().goal_node, state):
            print "Goal is reached"
            for pnode in plan:
                pnode.status = plans.ActionStatusEnum.EXECUTED
            return False

        #check for expandable assertions
        for pnode in plan:
            if isinstance(pnode, plans.DummyNode):
                continue

            if pnode.action.replan:
                if self.check_node(pnode, state, replan=True):
                    print "Assertion (%s %s) is expandable, triggering replanning." % (action.name, " ".join(a.name for a in pnode.full_args))
                    return True
        #print "time for checking assertions:", time.time()-t0

        print "checking plan validity."
        #print "current state is:", map(str, state.iterfacts())
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
                print "Action (%s %s) is not executable, trying to skip it." % (action.name, " ".join(a.name for a in pnode.full_args))
                # if an action is not executable, maybe it has already been executed
                # so we're trying if the rest of the plan is executable in the initial state
                skipped_actions = i
                state = task.get_state().copy()
            else:
                print "Action (%s %s) ok." % (action.name, " ".join(a.name for a in pnode.full_args))
                for f in pnode.effects:
                    state.set(f)
            #print "time for checking action (%s %s): %f" % (action.name, " ".join(a.name for a in pnode.full_args), time.time()-t1)

        t2 = time.time()
        #Now check if the goal is satisfied
        #print "state after execution is:", map(str, state.iterfacts())
        print "checking if goal is still satisfied."
        if self.check_node(task.get_plan().goal_node, state):
            if skipped_actions > -1:
                print "Skipped the first %d actions." % skipped_actions
                #newplan = task.get_plan().copy()
                for pnode in plan[0:skipped_actions]:
                   pnode.status = plans.ActionStatusEnum.EXECUTED
                #we skipped all actions and the goal ist still satisfied: done
                if skipped_actions > len(plan)-2:
                    task.get_plan().goal_node.status = plans.ActionStatusEnum.EXECUTED
                #task.set_plan(newplan)
            print "Plan is still valid."
            print "time for goal validation:", time.time()-t2
            print "total time for validation:", time.time()-t0
            return False

        print "time for goal validation:", time.time()-t2
        print "total time for validation:", time.time()-t0
        
        if first_invalid_action:
            print "Preconditions of (%s %s) are not satisfied, triggering replanning." % (first_invalid_action.action.name, " ".join(a.name for a in first_invalid_action.full_args))
        else:
            print "Goal isn't fulfilled by the current plan, triggering replanning."

        return True  # no monitoring currently

    
    def _start_planner(self, task):
        """
        Call base planner for this task.  The base planner should
        run in a separate thread or process to that this function
        can return immediately.  
        """
        # TODO: currently atomic process, ie planner will wait for result
        print "Planning was triggered for task %d." % task.taskID
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

def create_unique_dir(base_path, unique_dirname_fn, may_exist):
    """creates a new subdirectory in base_path. unique_dirname_fn is a
    function that produces a new, unique name every time it is called.
    create_unique_dir() loops until a directory name is produced that
    does not exist yet, creates the directory and returns its name.
    If may_exist is True, then uniqueness is NOT required. Instead a
    possibly existing directory is just cleaned.
    """
    while True:
        unique_id = unique_dirname_fn()
        tmp_dir = os.path.join(base_path, unique_id)
        if os.path.exists(tmp_dir):
            if may_exist:
                utils.removeall(tmp_dir)  # remove old version
            else:
                continue  # create a new, unique name
        if not os.path.exists(tmp_dir):
            os.makedirs(tmp_dir)
        return tmp_dir

class ContinualAxiomsFF(BasePlanner):
    """
    """
    PDDL_REXP = re.compile("\((.*)\)")
    def _prepare_input(self, task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        DEBUGGING = True
        if DEBUGGING:
            unique_dirname_fn = lambda: "static_dir_for_debugging"
        else:
            unique_dirname_fn = lambda: Planner.create_unique_planner_call_id("tmp")
        tmp_dir = create_unique_dir(planning_tmp_dir, unique_dirname_fn, may_exist=DEBUGGING)
        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "plan.pddl", "stdout.out")]
        pddl_strs = task.pddl_domain_str(), task.pddl_problem_str()
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
            print "Warning: FF did not find a plan or crashed.  FF output was:\n\n>>>"
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
        #action_list = [self.remove_inferable_vars(action, task._mapltask) for action in action_list]
        times_actions = enumerate(action_list)  # keep it sequentially for now
        plan = plan_postprocess.make_po_plan(times_actions, task)
#         nodes = [self.createPlanNode(a, t+1, task._mapltask) for t,a in times_actions]
#         for i in xrange(0, len(nodes)-1):
#             plan.add_node(nodes[i])
#             link = plans.OrderingConstraint(nodes[i], nodes[i+1])
#             plan.add_link(link)
#         if nodes:
#             plan.add_link(plan.init_node, nodes[0])
#             plan.add_link(nodes[-1], plan.goal_node)
#         else:
#             plan.add_link(plan.init_node, plan.goal_node)
        return plan

    def createPlanNode(self, action, time, task):
        elmts = action.split()
        action, args = elmts[0], elmts[1:]
        actions = dict((a.name,a) for a in itools.chain(task.actions, task.sensors))
        assert action in actions
        action_def = actions[action]
        args = [task[a] for a in args]
        return plans.PlanNode(action_def, args, time, plans.ActionStatusEnum.EXECUTABLE)
        
    
class TFD(BasePlanner):
    """
    """
    TFD_REXP = re.compile("([0-9\.]*): \((.*)\) \[([0-9\.]*)\]")
    def _prepare_input(self, task):
        planning_tmp_dir =  global_vars.config.tmp_dir
        DEBUGGING = True
        if DEBUGGING:
            unique_dirname_fn = lambda: "static_dir_for_debugging"
        else:
            unique_dirname_fn = lambda: Planner.create_unique_planner_call_id("tmp")
        tmp_dir = create_unique_dir(planning_tmp_dir, unique_dirname_fn, may_exist=DEBUGGING)
        paths = [os.path.join(tmp_dir, name) for name in ("domain.pddl", "problem.pddl", "stdout.out")]
        pddl_strs = task.tfd_domain_str(), task.tfd_problem_str()
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
        
        cmd = "%(translate)s %(domain_path)s %(problem_path)s" % locals()
        p_tr = utils.run_process(cmd, dir=tmp_dir)
        if p_tr.returncode != 0:
            print "Warning: Error in TFD translate. output was:\n\n>>>"
            print p_tr.stderr.read()
            print "<<<\n"
            return None
        p_pre = utils.run_process(preprocess, input=p_tr.stdout, dir=tmp_dir)
        if p_pre.returncode != 0:
            print "Warning: Error in TFD preprocess. output was:\n\n>>>"
            print p_pre.stderr.read()
            print "<<<\n"
            return None
        
        cmd = "%(search)s yY t 5 -" % locals()
        p_search = utils.run_process(cmd, input=p_pre.stdout, dir=tmp_dir)
        
        if p_search.returncode != 0:
            print "Warning: TFD search did not find a plan or crashed.  TFD output was:\n\n>>>"
            print p_search.stderr.read()
            print "<<<\n"
            return None

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
        #action_list = [self.remove_inferable_vars(action, task._mapltask) for action in action_list]
        times_actions = [(a[0], a[1]) for a in action_list]  # keep it sequentially for now
        plan = plan_postprocess.make_po_plan(times_actions, task)
        #nodes = [self.createPlanNode(a, t+1, task._mapltask) for t,a in times_actions]
        #for i in xrange(0, len(nodes)-1):
        #    plan.add_node(nodes[i])
        #    link = plans.OrderingConstraint(nodes[i], nodes[i+1])
        #    plan.add_link(link)
        #if nodes:
        #    plan.add_link(plan.init_node, nodes[0])
        #    plan.add_link(nodes[-1], plan.goal_node)
        #else:
        #    plan.add_link(plan.init_node, plan.goal_node)
            
        return plan
    
    def createPlanNode(self, action, time, task):
        elmts = action.split()
        action, args = elmts[0], elmts[1:]
        actions = dict((a.name,a) for a in itools.chain(task.actions, task.sensors))
        assert action in actions
        action_def = actions[action]
        args = [task[a] for a in args]
        return plans.PlanNode(action_def, args, time, plans.ActionStatusEnum.EXECUTABLE)
            
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
