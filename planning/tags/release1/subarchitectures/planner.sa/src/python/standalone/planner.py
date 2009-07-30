import os, sys
import re

import utils
import globals as global_vars

from task import PlanningStatusEnum
import plans

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
        replanning_necessary = self._evaluate_current_plan(task)
        if replanning_necessary:
            self._start_planner(task)

    def _evaluate_current_plan(self, task):
        """
        Plan monitoring: checks whether a plan is still valid given
        a modified task description. 
        Returns True if replanning is necessary, else False.
        """
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
        plan = self._post_process(output_data, task)
        return plan
    
    def _prepare_input(self, task):
        raise NotImplementedError
    
    def _run(self, task):
        raise NotImplementedError
    
    def _post_process(self, task):
        raise NotImplementedError

def create_unique_dir(base_path, unique_dirname_fn):
    """creates a new subdirectory in base_path. unique_dirname_fn is a
    function that produces a new, unique name every time it is called.
    create_unique_dir() loops until a directory name is produced that
    does not exist yet, creates the directory and returns its name."""
    while True:
        unique_id = unique_dirname_fn()
        print unique_id
        tmp_dir = os.path.join(base_path, unique_id)
        if not os.path.exists(tmp_dir):
            os.make_dirs(tmp_dir)
            return tmp_dir
    
class ContinualAxiomsFF(BasePlanner):
    """
    """
    PDDL_REXP = re.compile("\((.*)\)")
    def _prepare_input(self, task):
        planning_tmp_dir =  global_vars.config.tmp_dir
#         unique_dirname_fn = lambda: Planner.create_unique_planner_call_id("tmp")
#         tmp_dir = create_unique_dir(planning_tmp_dir, unique_dirname_fn, static_testing=True)
        DEBUGGING = True
        if DEBUGGING:
            tmp_dir = os.path.join(planning_tmp_dir, "static_dir_for_debugging")
            if not os.path.exists(tmp_dir):
                os.makedirs(tmp_dir)
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
        stdout_output = utils.run_command(cmd, output=stdout_path)
#         print "Planner output:"
#         print stdout_output
        pddl_output = open(plan_path).read()
        pddl_plan = self.parse_ff_output(pddl_output)
        return pddl_plan

    def parse_ff_output(self, pddl_output):
        lines = [line for line in pddl_output.splitlines() if line]
        result = lines.pop(0)
        assert result in ("SUCCESS", "FAILURE"), "wrong format for FF output"
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
        nodes = [plans.PlanNode(a, t+1) for t,a in times_actions]
        for i in xrange(0, len(nodes)-1):
            plan.add_node(nodes[i])
            link = plans.OrderingConstraint(nodes[i], nodes[i+1])
            plan.add_link(link)
        plan.add_link(plan.init_node, nodes[0])
        plan.add_link(nodes[-1], plan.goal_node)
        return plan
            
