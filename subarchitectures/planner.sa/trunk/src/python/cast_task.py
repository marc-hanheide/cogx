from os.path import abspath, dirname, join, isdir

import cast_state, dt_problem
from autogen import Planner
from standalone import task, plans, pddl, config
from standalone.task import PlanningStatusEnum

log = config.logger("PythonServer")

PLANNER_CP = 0
PLANNER_DT = 1

status_dict = {PlanningStatusEnum.TASK_CHANGED : Planner.Completion.PENDING, \
                   PlanningStatusEnum.RUNNING : Planner.Completion.INPROGRESS, \
                   PlanningStatusEnum.PLAN_AVAILABLE : Planner.Completion.SUCCEEDED, \
                   PlanningStatusEnum.PLANNING_FAILURE : Planner.Completion.FAILED, \
                   PlanningStatusEnum.INTERRUPTED : Planner.Completion.ABORTED }


class CASTTask(object):
    def __init__(self, planning_task, beliefs, domain_fn, component):
        self.component = component

        self.id = planning_task.id
        self.original_task = planning_task
        self.dt_task = None

        self.load_domain(domain_fn)

        self.state = cast_state.CASTState(beliefs, self.domain)

        cp_problem = self.state.to_problem(planning_task, deterministic=True, domain=self.cp_domain)
        self.cp_task = task.Task(self.id, cp_problem)
        component.planner.register_task(self.cp_task)
        
        self.update_status(Planner.Completion.PENDING)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % self.id))
        self.write_cp_problem(problem_fn)

        
    def update_status(self, status):
        self.status = status
        self.component.getClient().updateStatus(self.id, self.status)
        self.component.m_display.update_task(self)

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

    def write_plan(self):
        plan = self.get_plan()
        G = plan.to_dot()
        dot_fn = abspath(join(self.component.get_path(), "plan%d.dot" % self.id))
        G.write(dot_fn)
        log.debug("Dot file for plan is stored in %s", dot_fn)

        if self.component.show_dot:
            log.info("Showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh")
            show_dot_script = abspath(join(self.component.get_path(), "show_dot.sh"))
            os.system("%s %s" % (show_dot_script, dot_fn))

    def run(self):
        self.update_status(Planner.Completion.INPROGRESS)
        self.cp_task.replan()
        self.process_cp_plan()

    def process_cp_plan(self):
        plan = self.get_plan()

        if plan is None:
            self.update_status(Planner.Completion.FAILED)
            return

        if "partial-observability" in self.domain.requirements:
            log.info("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.domain, self.state)
            
            if self.dt_planning_active():
                self.update_status(Planner.Completion.INPROGRESS)
                self.component.start_dt_planning(self)
                return
            
        log.debug("The following plan was found %s:\n", plan)

        self.write_plan()

        ordered_plan = plan.topological_sort()
        outplan = []
        first_action = -1

        for i, pnode in enumerate(ordered_plan):
            if isinstance(pnode, plans.DummyNode) or not pnode.is_executable():
                continue
            if first_action == -1:
                first_action = i

            uargs = [self.state.featvalue_from_object(arg) for arg in pnode.args]

            fullname = str(pnode)
            outplan.append(Planner.Action(self.id, pnode.action.name, uargs, fullname, Planner.Completion.PENDING))

        if outplan:
            log.info("First action: %s == %s", str(ordered_plan[first_action]), outplan[0].fullName)
        else:
            log.info("Plan is empty")
        plan.execution_position = first_action

        self.update_status(Planner.Completion.SUCCEEDED)

        self.component.deliver_plan(self, outplan)

    def action_executed_dt(self, slice_plan):
        assert len(slice_plan) == 1
        dt_action = slice_plan[0]

        failed_actions = []
        finished_actions = []
        
        #Failed dt action causes all actions resulting in the subplan to fail
        if dt_action.status == Planner.Completion.ABORTED or Planner.Completion.FAILED:
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
                failed_actions.append(pnode)
            self.cp_task.mark_changed()
            failed_actions.append(self.dt_task.plan[-1])
        elif dt_action.status == Planner.Completion.SUCCEEDED:
            finished_actions.append(self.dt_task.plan[-1])
        
        if finished_actions or failed_actions:
            self.action_feedback(finished_actions, failed_actions)

        self.monitor_dt()
            
    def action_executed_cp(self, slice_plan):
        plan = self.get_plan()

        finished_actions = []
        failed_actions = []

        if plan is None:
            #always replan if we don't have a plan
            self.cp_task.mark_changed()
        else:
            log.debug("checking execution state")
            executable_plan = plan.topological_sort()[plan.execution_position:-1]

            if len(slice_plan) != len(executable_plan):
                log.error("wm plan:")
                for action in slice_plan:
                    log.error("%s, status: %s", action.fullName, str(action.status))
                log.error("internal plan (execution position is %d):", plan.execution_position)
                for pnode in plan.topological_sort():
                    log.error("%s, status: %s", str(pnode), pnode.status)
                raise Exception("Plans from WMControl and Planner don't match!")

            requires_action_dispatch = False
            for action, pnode in zip(slice_plan, executable_plan):
                if action.status != Planner.Completion.PENDING:
                    log.debug("status of %s is %s", action.fullName, str(action.status))

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
                    self.cp_task.mark_changed()

        if requires_action_dispatch:
            self.cp_task.mark_changed()

        if finished_actions or failed_actions:
            self.action_feedback(finished_actions, failed_actions)
            
        self.monitor_cp()

    def monitor_dt(self):
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            if any(fact not in self.state.state for fact in pnode.effects):
                sat = False
                break
        if sat:
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.EXECUTED
            self.cp_task.mark_changed()
            self.monitor_cp()
            return

        obs = Planner.Observation("predicate", ["arg1", "arg2"])
        self.component.getDT().deliverObservation(self.id, [obs])
        log.info("%d: delivered dummy observation", self.id)

        self.update_status(Planner.Completion.INPROGRESS)
  
    def monitor_cp(self):
        if task.dt_planning_active():
            self.monitor_dt()
            return
        
        self.update_status(Planner.Completion.INPROGRESS)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % self.id))
        self.write_cp_problem(problem_fn)

        self.cp_task.replan()
        self.process_cp_plan()

    def action_delivered(self, action):
        args = [self.cp_task.mapltask[a] for a in action.arguments]
        pddl_action = task.domain.get_action(action.name)

        state = self.cp_task.get_state().copy()
        pnode = standalone.plan_postprocess.getRWDescription(pddl_action, args, state, 1)
        self.dt_task.dt_plan.append(pnode)
        
        self.update_status(Planner.Completion.SUCCEEDED)
        
        #create featurevalues
        uargs = [self.state.featvalue_from_object(a) for a in args]
    
        fullname = action.name + " ".join(action.arguments)
        outplan = [Planner.Action(self.id, action.name, uargs, fullname, Planner.Completion.PENDING)]

        log.info("%d: First action: %s", taskId, fullname)
        self.component.deliver_plan(self, outplan)
        
    def action_feedback(self, finished_actions, failed_actions):
        diffstate = compute_state_updates(self.cp_task.get_state(), finished_actions, failed_actions)
        for fact in diffstate.iterfacts():
            self.cp_task.get_state().set(fact)
        #TODO: create new state?
        beliefs = self.state.update_beliefs(diffstate)
        self.component.getClient().updateBeliefState(beliefs)
                
    def update_state(self, beliefs):
        self.state = CASTState(beliefs, self.domain)
        new_cp_problem = self.state.to_problem(None, deterministic=True, domain=self.cp_domain)

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

        return plan
        
        
def compute_state_updates(_state, actions, failed):
    diffstate = pddl.state.State()
    for action in actions:
        for fact in action.effects:
            if fact.svar.modality != pddl.mapl.update:
                continue

            fact = pddl.state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])

            if fact not in _state:
                diffstate.set(fact)
                log.debug("not in state: %s", str(fact))
            elif fact.svar in diffstate:
                del diffstate[fact.svar]
                log.debug("previous change %s overwritten by later action", str(pddl.state.Fact(fact.svar, diffstate[fact.svar])))

    for action in failed:
        for fact in action.effects:
            if fact.svar.modality != pddl.mapl.update_fail:
                continue

            fact = pddl.state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])

            if fact not in _state:
                diffstate.set(fact)
                log.debug("not in state: %s", str(fact))
            elif fact.svar in diffstate:
                del diffstate[fact.svar]
                log.debug("previous change %s overwritten by later action", str(pddl.state.Fact(fact.svar, diffstate[fact.svar])))

    return diffstate
