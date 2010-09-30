import os
from os.path import abspath, dirname, join, isdir

from de.dfki.lt.tr.beliefs.slice import logicalcontent
from autogen import Planner

from standalone import task, plans, plan_postprocess, pddl, config
from standalone.task import PlanningStatusEnum
from standalone.utils import Enum

import cast_state, dt_problem

TaskStateEnum = Enum("INITIALISED",
                     "PROCESSING",
                     "WAITING_FOR_ACTION",
                     "WAITING_FOR_BELIEF",
                     "WAITING_FOR_DT",
                     "FAILED",
                     "COMPLETED")

status_dict = {TaskStateEnum.INITIALISED : Planner.Completion.PENDING, \
                   TaskStateEnum.PROCESSING : Planner.Completion.INPROGRESS, \
                   TaskStateEnum.WAITING_FOR_ACTION : Planner.Completion.PENDING, \
                   TaskStateEnum.WAITING_FOR_BELIEF : Planner.Completion.PENDING, \
                   TaskStateEnum.WAITING_FOR_DT : Planner.Completion.INPROGRESS, \
                   TaskStateEnum.FAILED : Planner.Completion.FAILED, \
                   TaskStateEnum.COMPLETED : Planner.Completion.SUCCEEDED }
                   
                   
log = config.logger("plan-control")

PLANNER_CP = 0
PLANNER_DT = 1

WAIT_FOR_ACTION_TIMEOUT = 2000

# status_dict = {PlanningStatusEnum.TASK_CHANGED : Planner.Completion.PENDING, \
#                    PlanningStatusEnum.RUNNING : Planner.Completion.INPROGRESS, \
#                    PlanningStatusEnum.PLAN_AVAILABLE : Planner.Completion.SUCCEEDED, \
#                    PlanningStatusEnum.PLANNING_FAILURE : Planner.Completion.FAILED, \
#                    PlanningStatusEnum.INTERRUPTED : Planner.Completion.ABORTED }

def fval_to_str(fval):
    if fval.__class__ == logicalcontent.ElementaryFormula:
        return "'%s'" % fval.prop
    elif fval.__class__ == logicalcontent.PointerFormula:
        return "%s@%s" % (fval.pointer.id, fval.pointer.subarchitecture)
    elif fval.__class__ == logicalcontent.IntegerFormula:
        return str(fval.val)
    elif fval.__class__ == logicalcontent.BooleanFormula:
        return str(fval.val)
    elif fval.__class__ == logicalcontent.UnknownFormula:
        return "UNKNOWN"
    assert False

class CASTTask(object):
    def __init__(self, planning_task, beliefs, domain_fn, component, problem_fn=None):
        self.component = component

        self.id = planning_task.id
        self.slice_goals = planning_task.goals
        self.dt_task = None
        self.step = 0

        self.load_domain(domain_fn)
        if problem_fn:
            import fake_cast_state
            log.info("Loading predefined problem: %s.", problem_fn)
            add_problem = pddl.load_problem(problem_fn, self.domain)
            self.state = fake_cast_state.FakeCASTState(add_problem, self.domain)
        else:
            self.state = cast_state.CASTState(beliefs, self.domain)
            
        self.percepts = []

        cp_problem, self.goaldict = self.state.to_problem(planning_task, deterministic=True, domain=self.cp_domain)
        
        self.cp_task = task.Task(self.id, cp_problem)
        self.waiting_for_action = False
        component.planner.register_task(self.cp_task)
        
        self.internal_state = TaskStateEnum.INITIALISED
        self.update_status(TaskStateEnum.INITIALISED)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % self.id))
        self.write_cp_problem(problem_fn)

        domain_out_fn = abspath(join(self.component.get_path(), "domain%d.mapl" % self.id))
        w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        w.write(self.cp_task.mapltask, domain_fn=domain_out_fn)
        
        
    def update_status(self, status):
        self.internal_state = status
        self.status = status_dict[status]
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
        self.update_status(TaskStateEnum.PROCESSING)
        self.cp_task.replan()
        self.process_cp_plan()

    def process_cp_plan(self):
        plan = self.get_plan()

        if plan is None:
            self.update_status(TaskStateEnum.FAILED)
            return

        for sg in plan.goal_node.satisfied_softgoals:
            slice_goal = self.goaldict[sg]
            slice_goal.isInPlan = True

        for g in self.slice_goals:
            if g.importance < 0 and g.goalString in self.goaldict:
                g.isInPlan = True
            log.debug("Goal: %s, p:%.2f, sat: %d", g.goalString, g.importance, g.isInPlan)

        if "partial-observability" in self.domain.requirements:
            log.info("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.domain, self.state)

            for pnode in plan.nodes_iter():
                if pnode.is_virtual():
                    pnode.status = plans.ActionStatusEnum.EXECUTED
            
            #self.update_status(self.status)
            if self.dt_planning_active():
                self.update_status(TaskStateEnum.WAITING_FOR_DT)
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

            fullname = "%s %s" % (pnode.action.name, " ".join(fval_to_str(a) for a in uargs))
            outplan.append(Planner.Action(self.id, pnode.action.name, uargs, fullname, float(pnode.cost), Planner.Completion.PENDING))

        if outplan:
            log.info("First action: %s == %s", str(ordered_plan[first_action]), outplan[0].fullName)
        else:
            log.info("Plan is empty")
        plan.execution_position = first_action

        self.internal_state = TaskStateEnum.WAITING_FOR_ACTION
        self.component.deliver_plan(self, outplan)

    def action_executed_dt(self, slice_plan):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF)
        
        assert len(slice_plan) == 1
        dt_action = slice_plan[0]

        failed_actions = []
        finished_actions = []

        #Failed dt action causes all actions resulting in the subplan to fail
        if dt_action.status == Planner.Completion.ABORTED or dt_action.status == Planner.Completion.FAILED:
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
                failed_actions.append(pnode)
            self.cp_task.mark_changed()
            failed_actions.append(self.dt_task.dt_plan[-1])
        elif dt_action.status == Planner.Completion.SUCCEEDED:
            finished_actions.append(self.dt_task.dt_plan[-1])
        
        if finished_actions or failed_actions:
            self.action_feedback(finished_actions, failed_actions)

        if failed_actions:
            self.monitor_cp()
            return
        self.monitor_dt()
            
    def action_executed_cp(self, slice_plan):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF)
        
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
                elif action.status in (Planner.Completion.ABORTED, Planner.Completion.FAILED):
                    pnode.status = plans.ActionStatusEnum.FAILED
                    failed_actions.append(pnode)
                    self.cp_task.mark_changed()

            if requires_action_dispatch:
                self.cp_task.mark_changed()

        if finished_actions or failed_actions:
            self.action_feedback(finished_actions, failed_actions)
            
        self.monitor_cp()

    def monitor_dt(self, pending_updates=False):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF)
        
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            if any(fact not in self.state.state for fact in pnode.effects):
                sat = False
                break
        if sat:
            self.dt_done()

        observations = []
        for fact in self.state.convert_percepts(self.percepts):
            pred = "observed-%s" % fact.svar.function.name
            obs = Planner.Observation(pred, [a.name for a in fact.svar.args] + [fact.value.name])
            observations.append(obs)
            log.info("%d: delivered observation (%s %s)", self.id, obs.predicate, " ".join(a for a in obs.arguments))

        if not observations:
            if not pending_updates and self.internal_state == TaskStateEnum.WAITING_FOR_BELIEF:
                # dummy observation as the dt planner terminates upon getting an empty observation
                log.info("Got no observations from %s", str(self.dt_task.dt_plan[-1]))
                observations.append(Planner.Observation("null", []))
            elif not pending_updates and self.internal_state != TaskStateEnum.WAITING_FOR_BELIEF:
                #TODO: only do the waiting if we really expect an observation (e.g. not for move)
                log.info("Waiting for observations from %s", str(self.dt_task.dt_plan[-1]))
                self.update_status(TaskStateEnum.WAITING_FOR_BELIEF)
                self.component.getClient().waitForChanges(self.id, WAIT_FOR_ACTION_TIMEOUT/2)
                return
            elif pending_updates:
                log.info("Still waiting for observations from %s...", str(self.dt_task.dt_plan[-1]))
                return
            
        self.update_status(TaskStateEnum.WAITING_FOR_DT)
        self.component.getDT().deliverObservation(self.id, observations)
  
    def monitor_cp(self, pending_updates=False):
        assert self.internal_state in (TaskStateEnum.PROCESSING, TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF)
        if pending_updates:
            assert self.internal_state == TaskStateEnum.WAITING_FOR_BELIEF
            
        if self.dt_planning_active():
            self.process_cp_plan()
            return

        if not pending_updates and self.waiting_for_action:
            #timeout reached, give up
            self.cp_task.set_plan(None, update_status=True)
            self.update_status(TaskStateEnum.FAILED)
            return

        self.update_status(TaskStateEnum.PROCESSING)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % (self.id)))
        self.write_cp_problem(problem_fn)

        self.step += 1
        self.cp_task.replan()
        if self.cp_task.planning_status == PlanningStatusEnum.WAITING:
            self.update_status(TaskStateEnum.WAITING_FOR_BELIEF)
            if pending_updates:
                log.info("Still waiting for effects of %s to appear", str(self.cp_task.pending_action))
                return
            log.info("Waiting for effects of %s to appear", str(self.cp_task.pending_action))
            self.component.getClient().waitForChanges(self.id, WAIT_FOR_ACTION_TIMEOUT)
            return
        
        self.waiting_for_action = False
        self.process_cp_plan()

    def dt_done(self):
        first_action = -1
        dt_action_found = False
        for i,pnode in enumerate(self.get_plan().topological_sort()):
            if pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.EXECUTED
                dt_action_found = True
            elif dt_action_found:
                first_action = i
        self.get_plan().execution_position = first_action
        self.update_status(TaskStateEnum.PROCESSING)

        self.cp_task.mark_changed()
        self.monitor_cp()

    def action_delivered(self, action):
        assert self.internal_state == TaskStateEnum.WAITING_FOR_DT
        
        log.debug("raw action: (%s %s)", action.name, " ".join(action.arguments))
        args = [self.cp_task.mapltask[a] for a in action.arguments]
        pddl_action = self.dt_task.dtdomain.get_action(action.name)

        log.debug("got action from DT: (%s %s)", action.name, " ".join(action.arguments))
        #log.debug("state is: %s", self.cp_task.get_state())

        if pddl_action.name in set(a.name for a in self.dt_task.goal_actions):
            log.info("Goal action recieved. DT task completed")
            self.dt_done()
            return
        
        state = self.cp_task.get_state().copy()
        #TODO: using the last CP state might be problematic

        pnode = plan_postprocess.getRWDescription(pddl_action, args, state, 1)
        
        self.dt_task.dt_plan.append(pnode)

        self.percepts = []
        
        #create featurevalues
        uargs = [self.state.featvalue_from_object(a) for a in pnode.args]
    
        fullname = "%s %s" % (pnode.action.name, " ".join(fval_to_str(a) for a in uargs))
        outplan = [Planner.Action(self.id, action.name, uargs, fullname, float(pnode.cost), Planner.Completion.PENDING)]

        log.info("%d: First action: %s", self.id, fullname)
        
        self.internal_state = TaskStateEnum.WAITING_FOR_ACTION
        self.component.deliver_plan(self, outplan)
        
    def action_feedback(self, finished_actions, failed_actions):
        diffstate = compute_state_updates(self.cp_task.get_state(), finished_actions, failed_actions)
        for fact in diffstate.iterfacts():
            self.cp_task.get_state().set(fact)
        #TODO: create new state?
        beliefs = self.state.update_beliefs(diffstate)
        self.component.getClient().updateBeliefState(beliefs)
                
    def update_state(self, beliefs):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF, TaskStateEnum.FAILED)
        
        import fake_cast_state
        if isinstance(self.state, fake_cast_state.FakeCASTState):
            return True
        
        self.state = cast_state.CASTState(beliefs, self.domain, self.state)
        new_cp_problem, _ = self.state.to_problem(None, deterministic=True, domain=self.cp_domain)

        #check if the goal is still valid
        try:
            new_cp_problem.goal = self.cp_task._mapltask.goal.copy(new_cp_problem)
        except KeyError:
            log.warning("Goal is not valid anymore.")
            new_cp_problem.goal = pddl.conditions.Falsity()
            #self.cp_task.set_state(Planner.Completion.PLANNING_FAILURE)
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
