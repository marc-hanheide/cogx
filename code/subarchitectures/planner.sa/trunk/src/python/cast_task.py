import cast_state, dt_problem
from autogen import Planner
from standalone import task, pddl, config
from standalone.task import PlanningStatusEnum

log = config.logger("PythonServer")

PLANNER_CP = 0
PLANNER_DT = 1

class CASTTask(object):
    def __init__(self, planning_task, beliefs, domain_fn):
        self.id = planning_task.id
        self.orginial_task = planning_task
        self.dt_task = None

        self.load_domain(domain_fn)

        self.state = cast_state.CASTState(beliefs, self.domain)

        cp_problem = self.state.to_problem(planning_task, deterministic=True, domain=self.cp_domain)
        self.cp_task = task.Task(self.id, cp_problem)

    def load_domain(self, domain_fn):
        log.info("Loading domain %s.", domain_fn)
        self.domain = pddl.load_domain(domain_fn)

        if "partial-observability" in self.domain.requirements:
            self.cp_domain = pddl.dtpddl.DT2MAPLCompiler().translate(self.domain)
        else:
            self.cp_domain = self.domain

    def write_cp_problem(self, problem_fn):
        w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        w.write(self.cp_task.mapltask, problem_fn=problem_fn)

    def replan(self):
        self.cp_task.replan()

    def mark_changed(self):
        self.cp_task.mark_changed()
        
    def update_task(self, beliefs):
        self.state = CASTState(beliefs, self.domain)
        new_cp_problem = self.state.to_problem(planning_task, deterministic=True, domain=self.cp_domain)

        #check if the goal is still valid
        try:
            new_cp_problem.goal = self.cp_task._mapltask.goal.copy(new_cp_problem)
        except KeyError:
            log.warning("Goal is not valid anymore.")
            new_cp_problem.goal = pddl.conditions.Falsity()
            self.cp_task.set_state(Planner.Completion.PLANNING_FAILURE)
            return False
        
        self.cp_task.mapltask = new_cp_problem
        self.cp_task.set_state(self.state.state)
        
        return True

    def dt_planning_active(self):
        plan = self.get_plan()
        if not plan or not self.dt_task:
            return False
        
        return self.dt_task.subplan_active(plan)

    def get_plan(self):
        plan = self.cp_task.get_plan()

        if plan is None or self.cp_task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
            return None

        if "partial-observability" in self.domain.requirements:
            log.info("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.domain, self.state)

        return plan
        
        
