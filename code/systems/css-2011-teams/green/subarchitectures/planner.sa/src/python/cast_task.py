import os
from os.path import abspath, join

from de.dfki.lt.tr.beliefs.slice import logicalcontent
from autogen import Planner

from standalone import task, dt_problem, plans, plan_postprocess, pddl, config
import standalone.globals as global_vars
from standalone.task import PlanningStatusEnum
from standalone.utils import Enum

import cast_state

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
        self.dt_id = None
        self.slice_goals = planning_task.goals
        self.dt_task = None
        self.step = 0
        self.plan_history = []

        self.load_domain(domain_fn)
        if problem_fn:
            import fake_cast_state
            log.info("Loading predefined problem: %s.", problem_fn)
            add_problem = pddl.load_problem(problem_fn, self.domain)
            self.state = fake_cast_state.FakeCASTState(add_problem, self.domain, component=component)
        else:
            self.state = cast_state.CASTState(beliefs, self.domain, component=component)
            
        self.percepts = []

        cp_problem, self.goaldict = self.state.to_problem(planning_task.goals, deterministic=True, domain=self.cp_domain)
        for g in self.slice_goals:
            if g.importance == -1 and g.goalString not in self.goaldict:
                log.info("Hard goal %s cannot be parsed; planning failed" % g.goalString)
                self.update_status(TaskStateEnum.FAILED)
                return
        
        self.cp_task = task.Task(self.id, cp_problem)
        component.planner.register_task(self.cp_task)

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

    def wait(self, timeout, update_callback, timeout_callback):
        self.wait_update_callback = update_callback
        self.wait_timeout_callback = timeout_callback
        self.update_status(TaskStateEnum.WAITING_FOR_BELIEF)
        self.component.getClient().waitForChanges(self.id, timeout)

    def wait_update(self):
        assert self.wait_update_callback is not None
        if self.wait_update_callback():
            self.wait_update_callback = None
            self.wait_timeout_callback = None

    def wait_timeout(self):
        if self.wait_timeout_callback is None:
            return
        self.wait_timeout_callback()
        self.wait_timeout_callback = None

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

    def retry(self):
        # self.state = cast_state.CASTState(beliefs, self.domain, component=self.component)
        # self.percepts = []

        # cp_problem, self.goaldict = self.state.to_problem(planning_task, deterministic=True, domain=self.cp_domain)
        
        # self.cp_task = task.Task(self.id, cp_problem)
        # component.planner.register_task(self.cp_task)
        
        # self.update_status(TaskStateEnum.INITIALISED)

        # problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % self.id))
        # self.write_cp_problem(problem_fn)

        # domain_out_fn = abspath(join(self.component.get_path(), "domain%d.mapl" % self.id))
        # w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        # w.write(self.cp_task.mapltask, domain_fn=domain_out_fn)
        self.cp_task.mark_changed()
        self.update_status(TaskStateEnum.PROCESSING)
        self.cp_task.replan()
        self.process_cp_plan()

    def process_cp_plan(self):
        plan = self.get_plan()

        if plan is None:
            self.plan_history.append(plan)
            self.update_status(TaskStateEnum.FAILED)
            return

        for sg in plan.goal_node.satisfied_softgoals:
            if sg in self.goaldict:
                slice_goal = self.goaldict[sg]
                slice_goal.isInPlan = True

        for g in self.slice_goals:
            if g.importance < 0 and g.goalString in self.goaldict:
                g.isInPlan = True
            log.debug("Goal: %s, p:%.2f, sat: %d", g.goalString, g.importance, g.isInPlan)

        if "partial-observability" in self.domain.requirements:
            log.info("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.domain)

            for pnode in plan.nodes_iter():
                if pnode.is_virtual():
                    pnode.status = plans.ActionStatusEnum.EXECUTED
            
            #self.update_status(self.status)
            if self.dt_planning_active():
                self.dt_task.initialize(self.state.prob_state)
                self.update_status(TaskStateEnum.WAITING_FOR_DT)
                self.component.start_dt_planning(self)
                return
            
        log.debug("The following plan was found %s:\n", plan)

        self.write_plan()

        ordered_plan = plan.topological_sort()
        exec_plan = []
        first_action = -1

        for i, pnode in enumerate(ordered_plan):
            if isinstance(pnode, plans.DummyNode) or not pnode.is_executable():
                continue
            if first_action == -1:
                first_action = i
            exec_plan.append(pnode)
            
        plan.execution_position = first_action
        self.dispatch_actions(exec_plan)

    def action_executed_dt(self, slice_plan):
        assert self.internal_state == TaskStateEnum.WAITING_FOR_ACTION
        
        assert len(slice_plan) == 1
        dt_action = slice_plan[0]
        dt_pnode = self.dt_task.dt_plan[-1]

        failed_actions = []
        finished_actions = []

        #Failed dt action causes all actions in the subplan to fail
        if dt_action.status == Planner.Completion.ABORTED or dt_action.status == Planner.Completion.FAILED:
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
                failed_actions.append(pnode)
            self.cp_task.mark_changed()
            failed_actions.append(dt_pnode)
            dt_pnode.status = plans.ActionStatusEnum.FAILED
        elif dt_action.status == Planner.Completion.SUCCEEDED:
            finished_actions.append(dt_pnode)
            dt_pnode.status = plans.ActionStatusEnum.EXECUTED
        
        if finished_actions or failed_actions:
            self.action_feedback(finished_actions, failed_actions)

        if failed_actions:
            self.monitor_cp()
            return
        self.monitor_dt()
            
    def action_executed_cp(self, slice_plan):
        assert self.internal_state == TaskStateEnum.WAITING_FOR_ACTION
        
        plan = self.get_plan()

        finished_actions = []
        failed_actions = []

        if plan is None:
            #always replan if we don't have a plan
            self.cp_task.mark_changed()
        else:
            log.debug("checking execution state")
            executable_plan = [pnode for pnode in plan.topological_sort()[plan.execution_position:-1] if not pnode.is_virtual()]

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

    def monitor_dt(self):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION)
        
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            if any(fact not in self.state.state for fact in pnode.effects):
                sat = False
                break
        if sat:
            self.dt_done()

        dt_pnode = self.dt_task.dt_plan[-1]

        if self.dt_task.replanning_neccessary(self.state.prob_state):
            log.info("DT task requires replanning")
            #send empty observations to terminate previous task
            self.component.cancel_dt_session(self)
            self.dt_task.recompute_problem(self.state.prob_state)
            self.update_status(TaskStateEnum.WAITING_FOR_DT)
            self.component.start_dt_planning(self)

        def get_observations():
            result = []
            for fact in self.state.convert_percepts(self.percepts):
                pred = "observed-%s" % fact.svar.function.name
                obs = Planner.Observation(pred, [a.name for a in fact.svar.args] + [fact.value.name])
                result.append(obs)
                log.info("%d: delivered observation (%s %s)", self.id, obs.predicate, " ".join(a for a in obs.arguments))
            return result

        def wait_for_observation():
            observations = get_observations()
            if not observations:
                log.info("Still waiting for observations from %s...", str(self.dt_task.dt_plan[-1]))
                return False
            self.update_status(TaskStateEnum.WAITING_FOR_DT)
            self.component.getDT().deliverObservation(self.dt_id, observations)
            return True

        def wait_timeout():
            log.info("Got no observations from %s", str(self.dt_task.dt_plan[-1]))
            observations = [Planner.Observation("null", [])]
            self.update_status(TaskStateEnum.WAITING_FOR_DT)
            self.component.getDT().deliverObservation(self.dt_id, observations)
            
        observations = get_observations()
        if not self.dt_task.observation_expected(dt_pnode.action):
            log.info("No observations expected from %s", str(self.dt_task.dt_plan[-1]))
            observations.append(Planner.Observation("null", []))
        elif not observations:
            log.info("Waiting for observations from %s", str(self.dt_task.dt_plan[-1]))
            self.wait(WAIT_FOR_ACTION_TIMEOUT/2, wait_for_observation, wait_timeout)
            return

        log.debug("delivered observations")
        self.update_status(TaskStateEnum.WAITING_FOR_DT)
        self.component.getDT().deliverObservation(self.dt_id, observations)
  
    def monitor_cp(self):
        assert self.internal_state in (TaskStateEnum.PROCESSING, TaskStateEnum.WAITING_FOR_ACTION)
            
        # if self.dt_planning_active():
        #     self.process_cp_plan()
        #     return

        self.update_status(TaskStateEnum.PROCESSING)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.mapl" % (self.id)))
        self.write_cp_problem(problem_fn)
        plan = self.cp_task.get_plan()

        def wait_for_effect():
            self.cp_task.replan()
            if self.cp_task.get_plan() != plan:
                self.plan_history.append(plan)
            if self.cp_task.planning_status == PlanningStatusEnum.WAITING:
                log.info("Still waiting for effects of %s to appear", str(self.cp_task.pending_action))
                return False
            self.process_cp_plan()
            return True

        def wait_timeout():
            log.info("Wait for %s timed out. Plan failed.", str(self.cp_task.pending_action))
            self.plan_history.append(plan)
            self.cp_task.set_plan(None, update_status=True)
            self.update_status(TaskStateEnum.FAILED)
            return

        self.step += 1
        self.cp_task.replan()
        if self.cp_task.get_plan() != plan:
            self.plan_history.append(plan)
            
        if self.cp_task.planning_status == PlanningStatusEnum.WAITING:
            log.info("Waiting for effects of %s to appear", str(self.cp_task.pending_action))
            self.wait(WAIT_FOR_ACTION_TIMEOUT, wait_for_effect, wait_timeout)
            return
        
        self.process_cp_plan()

    def dt_done(self):
        last_dt_action = -1
        dt_action_found = False
        for i,pnode in enumerate(self.get_plan().topological_sort()):
            if pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.EXECUTED
                dt_action_found = True
                last_dt_action = i
            elif dt_action_found:
                break

        log.info("dt planning cancelled.")
        self.component.cancel_dt_session(self)
            
        self.get_plan().execution_position = last_dt_action
        self.update_status(TaskStateEnum.PROCESSING)
        self.plan_history.append(self.dt_task)

        # import debug
        # debug.set_trace()

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
        

        def wait_for_effect():
            #TODO: using the last CP state might be problematic
            state = self.cp_task.get_state().copy()
            try:
                pnode = plan_postprocess.getRWDescription(pddl_action, args, state, 1)
            except:
                log.info("Still waiting for effects from previous action...")
                return False
            self.percepts = []
            self.dt_task.dt_plan.append(pnode)
            self.dispatch_actions([pnode])
            return True

        def wait_timeout():
            self.plan_history.append(self.dt_task)
            self.cp_task.set_plan(None, update_status=True)
            self.component.cancel_dt_session(self)
            self.update_status(TaskStateEnum.FAILED)
            return
        
        #TODO: using the last CP state might be problematic
        state = self.cp_task.get_state().copy()
        try:
            pnode = plan_postprocess.getRWDescription(pddl_action, args, state, 1)
        except:
            log.info("Action (%s %s) not executable. Waiting for action effects from previous action...", pddl_action.name, " ".join(action.arguments))
            self.wait(WAIT_FOR_ACTION_TIMEOUT, wait_for_effect, wait_timeout)
            return
            
        self.percepts = []
        self.dt_task.dt_plan.append(pnode)
        self.dispatch_actions([pnode])
        

    def dispatch_actions(self, nodes):
        outplan = []
        for pnode in nodes:
            uargs = [self.state.featvalue_from_object(a) for a in pnode.args]
            fullname = "%s %s" % (pnode.action.name, " ".join(fval_to_str(a) for a in uargs))
            outplan.append(Planner.Action(self.id, pnode.action.name, uargs, fullname, float(pnode.cost), Planner.Completion.PENDING))

        if outplan:
            log.info("First action: %s == %s", str(nodes[0]), outplan[0].fullName)
            nodes[0].status = plans.ActionStatusEnum.IN_PROGRESS
            self.update_status(TaskStateEnum.WAITING_FOR_ACTION)
        else:
            log.info("Plan is empty")
            self.update_status(TaskStateEnum.COMPLETED)
        
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
        
        self.state = cast_state.CASTState(beliefs, self.domain, self.state, component=self.component)
        new_cp_problem, self.goaldict = self.state.to_problem(self.slice_goals, deterministic=True, domain=self.cp_domain)
        for g in self.slice_goals:
            if g.importance == -1 and g.goalString not in self.goaldict:
                log.info("Hard goal %s cannot be parsed; planning failed" % g.goalString)
                return False

        # #check if the goal is still valid
        # try:
        #     new_cp_problem.goal = self.cp_task._mapltask.goal.copy(new_cp_problem)
        # except KeyError:
        #     log.warning("Goal is not valid anymore.")
        #     new_cp_problem.goal = pddl.conditions.Falsity()
        #     #self.cp_task.set_state(Planner.Completion.PLANNING_FAILURE)
        #     return False
        
        self.cp_task.mapltask = new_cp_problem
        self.cp_task.set_state(self.state.state)
        
        return True

    def dt_planning_active(self):
        if global_vars.config.dt.enabled == False:
            return False
        
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
