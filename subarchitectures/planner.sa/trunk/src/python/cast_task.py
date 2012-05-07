import os, re, time, itertools
from os.path import abspath, join
from collections import defaultdict

from de.dfki.lt.tr.beliefs.slice import logicalcontent
from eu.cogx.beliefs.slice import HypotheticalBelief
from autogen import Planner
import cast.cdl

from standalone import task, dt_problem, plans, plan_postprocess, pddl, config, domain_query_graph
import standalone.globals as global_vars
from standalone.task import PlanningStatusEnum
from standalone.utils import Enum, Struct
from standalone.pddl import state

import cast_state, planner_log
import explanations

TaskStateEnum = Enum("INITIALISED",
                     "PROCESSING",
                     "WAITING_FOR_ACTION",
                     "WAITING_FOR_BELIEF",
                     "WAITING_FOR_DT",
                     "FAILED",
                     "COMPLETED")

TaskStateInfoEnum = Enum("INITIALISED",
                         "PLANNING_CP",
                         "PLANNING_DT",
                         "WAITING_FOR_ACTION",
                         "WAITING_FOR_CONSISTENT_STATE",
                         "WAITING_FOR_EFFECT",
                         "WAITING_FOR_OBSERVATION",
                         "EXECUTION_FAILURE",
                         "PLANNING_FAILURE",
                         "NO_CONSISTENT_STATE",
                         "INVALID_GOAL",
                         "EXPLANATIONS_PENDING",
                         "EXPLANATIONS_FOUND",
                         "EXPLANATIONS_NOT_FOUND",
                         "COMPLETED")

status_dict = {TaskStateEnum.INITIALISED : Planner.Completion.PENDING, \
                   TaskStateEnum.PROCESSING : Planner.Completion.INPROGRESS, \
                   TaskStateEnum.WAITING_FOR_ACTION : Planner.Completion.PENDING, \
                   TaskStateEnum.WAITING_FOR_BELIEF : Planner.Completion.PENDING, \
                   TaskStateEnum.WAITING_FOR_DT : Planner.Completion.INPROGRESS, \
                   TaskStateEnum.FAILED : Planner.Completion.FAILED, \
                   TaskStateEnum.COMPLETED : Planner.Completion.SUCCEEDED }

action_status_dict = {plans.ActionStatusEnum.EXECUTABLE : Planner.Completion.PENDING,
                      plans.ActionStatusEnum.IN_PROGRESS : Planner.Completion.INPROGRESS,
                      plans.ActionStatusEnum.EXECUTED : Planner.Completion.SUCCEEDED,
                      plans.ActionStatusEnum.FAILED : Planner.Completion.FAILED}
                   
log = config.logger("plan-control")

PLANNER_CP = 0
PLANNER_DT = 1

WAIT_FOR_ACTION_TIMEOUT = 2000
WAIT_FOR_CONSISTENCY_TIMEOUT = 10000

TO_OK = -2
TO_WAIT = -3

FAKE_ACTION_FAILURE = None#"search_for_object_in_room"

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

class TimedOut(Exception):
    pass

def timeout(to):
    yield to
    while True:
        yield TO_WAIT

def coroutine(fn):
    def decorated_method(self, *args, **kwargs):
        cofn = fn(self, *args, **kwargs)
        try:
            while True:
                res = cofn.next()
                if res >= 0:
                    self.wait_co(cofn, res)
                    break
                elif res != TO_OK:
                    cofn.throw(AssertionError("Cofunction needs to yield a positive number (timeout) or TO_OK"))
                    
        except StopIteration:
            pass
        
    decorated_method.__name__ = fn.__name__
    decorated_method.__dict__ = fn.__dict__
    decorated_method.__doc__ = fn.__doc__
    return decorated_method

    
class CASTTask(object):
    def __init__(self, planning_task, beliefs, domain_fn, component, **args):
        self.component = component
        self.init_state = None
        self.id = planning_task.id
        self.dt_id = None
        self.slice_goals = planning_task.goals
        self.dt_task = None
        self.waiting_cofn = None
        self.step = 0
        self.plan_history = []
        self.plan_state_history = []
        self.fail_count = defaultdict(lambda: 0)
        self.failure_simulated = False
        self.expl_rules_fn = args.get('expl_rules_fn', None)
        log_fn = abspath(join(self.component.get_path(), 'planner-log-%d.log' % self.id))
        self.plan_log = planner_log.PlannerLog(log_fn)

        self.load_domain(domain_fn)

        if component.consistency_fn:
            self.consistency_cond = pddl.parser.Parser.parse_as(open(component.consistency_fn), pddl.Conjunction, self.domain)
        else:
            self.consistency_cond = None
            
        if component.history_fn:
            self.load_history(component.history_fn, component)
            self.failure_simulated = True
        elif args.get('problem_fn', None):
            import fake_cast_state, history
            log.info("Loading predefined problem: %s.", problem_fn)
            add_problem = pddl.load_problem(problem_fn, self.domain)
            self.state = fake_cast_state.FakeCASTState(add_problem, self.domain, component=component, consistency_cond=self.consistency_cond)
            self.init_state = self.state

            self.slice_goals = history.slice_goals_from_problem(add_problem)
        else:
            self.state = cast_state.CASTState(beliefs, self.domain, component=component, consistency_cond=self.consistency_cond)
            self.init_state = self.state
            
        self.plan_log.append(planner_log.StateEntry(self.state, self.slice_goals))
        self.percepts = []

        cp_problem, cp_domain, self.goaldict = self.state.to_problem(self.slice_goals, deterministic=True)
        self.cp_task = task.Task(self.id, cp_problem)

        deadline = -1
        for g in self.slice_goals:
            if g.importance == -1 and g.goalString not in self.goaldict:
                log.info("Hard goal %s cannot be parsed; planning failed" % g.goalString)
                self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.INVALID_GOAL )
                return
            if g.deadline != -1:
                if g.importance != -1:
                    log.error("Deadline for softgoal %s not supported. Will be ignored" % g.goalString)
                elif deadline == -1 :
                    deadline = g.deadline
                else:
                    deadline = min(deadline, g.deadline)

        self.cp_task.deadline = deadline
        component.planner.register_task(self.cp_task)

        self.unary_ops = None
        self.compute_solution_likelihood()
        
        self.update_status(TaskStateEnum.INITIALISED, TaskStateInfoEnum.INITIALISED)

        problem_fn = abspath(join(self.component.get_path(), "problem%d-init.pddl" % self.id))
        self.write_cp_problem(problem_fn)

        domain_out_fn = abspath(join(self.component.get_path(), "domain%d.pddl" % self.id))
        w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        w.write(self.cp_task.mapltask, domain_fn=domain_out_fn)

        log.debug("Planning task created: %.2f sec", global_vars.get_time())
        
        
    def update_status(self, status, info_status):
        self.internal_state = status
        self.status = status_dict[status]
        self.component.updateTaskStatus(self)
        self.component.m_display.update_task(self, info_status=info_status)
        self.component.m_display.update_state(self)

    def update_info_status(self, info_status):
        self.component.m_display.update_task(self, info_status=info_status)
        
    def load_domain(self, domain_fn):
        log.info("Loading domain %s.", domain_fn)
        domain = pddl.load_domain(domain_fn)
        t = pddl.translators.RemoveTimeCompiler();
        self.domain = t.translate(domain)
        if global_vars.config.enable_switching_planner:
            self.domain.dt_rules = [r.copy(self.domain) for r in domain.dt_rules]
            
            t0 = time.time()
            self.qgraph = domain_query_graph.QueryGraph(self.domain.dt_rules, self.domain)
            # print "\n\n------------------------------------------------------------------------------------\n\n"
            # self.action_qgraph = domain_query_graph.QueryGraph(self.domain.actions, self.domain)
            # self.action_qgraph = domain_query_graph.QueryGraph(self.domain.dt_rules + self.domain.actions, self.domain)
            log.debug("total time for constructing query graph: %f", time.time()-t0)

    def load_history(self, history_fn, component):
        import history
        log.info("Loading history: %s.", history_fn)
        for plan, state in history.load_history(history_fn, self.domain, component):
            if self.init_state is None:
                self.init_state = state
            self.state = state
            self.slice_goals = state.goals
                                                
            self.plan_history.append(plan)
            self.plan_state_history.append((plan, None, state))

    def wait_co(self, cofn, timeout):
        self.waiting_cofn = cofn
        log.debug("%s is waiting for updates. Timeout is %d ms", cofn.gi_code.co_name, timeout)
        self.component.getClient().waitForChanges(self.id, timeout)

    def wait_co_update(self):
        cofn = self.waiting_cofn
        log.debug("continuing %s after belief update.", cofn.gi_code.co_name)
        try:
            while True:
                res = cofn.next()
                if res == TO_WAIT:
                    log.debug("update was not handeled by %s, waiting some more", cofn.gi_code.co_name)
                    return
                elif res == TO_OK:
                    self.waiting_cofn = None
                    log.debug("update was handeled by %s", cofn.gi_code.co_name)
                else:
                    self.wait_co(cofn, res)
                    break
            
        except StopIteration:
            self.waiting_cofn = None
        
    def wait_co_timeout(self):
        cofn = self.waiting_cofn
        self.waiting_cofn = None
        log.debug("%s has timed out", cofn.gi_code.co_name)
        try:
            cofn.throw(TimedOut())
            while True:
                res = cofn.next()
                if res >= 0:
                    self.wait_co(cofn, res)
                    break
                elif res != TO_OK:
                    cofn.throw(AssertionError("Cofunction needs to yield a positive number (timeout) or TO_OK"))
        except StopIteration:
            pass

    def wait_update(self):
        if self.waiting_cofn is not None:
            return self.wait_co_update()
        log.debug("Task %d is listening for updates but no handler registered. Returning.", self.id)

    def wait_timeout(self):
        if self.waiting_cofn is not None:
            return self.wait_co_timeout()
        log.debug("Task %d timed out but no handler registered. Returning.", self.id)

    def write_cp_problem(self, problem_fn):
        init_prob, _, _ = self.state.to_problem(self.slice_goals, deterministic=False, raw_problem=True)
        w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        w.write(init_prob, problem_fn=problem_fn)

    def write_plan(self):
        plan = self.get_plan()
        G = plan.to_dot()
        dot_fn = abspath(join(self.component.get_path(), "plan%d.dot" % self.id))
        G.write(dot_fn)
        log.debug("Dot file for plan is stored in %s", dot_fn)

        if self.component.show_dot:
            show_dot_script = abspath(join(self.component.get_path(), "show_dot.sh"))
            os.system("%s %s" % (show_dot_script, dot_fn))

    def write_history(self):
        history_fn = abspath(join(self.component.get_path(), "history-%d.pddl" % self.id))
        f = open(history_fn, "w")
        log.debug("writing history to %s", history_fn)
        
        for plan, po_plan, state in self.plan_state_history:
            problem, _, _ = state.to_problem(self.slice_goals, deterministic=False)
            w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
            _, prob_str = w.write(problem)
            for l in prob_str:
                f.write(l)
                f.write("\n")
            f.write("END_PROBLEM\n")
            if plan is not None:
                for pnode in plan.topological_sort():
                    s = "(%s %s), %s" % (pnode.action.name, " ".join(a.name for a in pnode.full_args), pnode.status)
                    f.write(s)
                    f.write("\n")
            f.write("END_PLAN\n")
            for s in self.poplan_to_string(po_plan):
                f.write(s)
                f.write("\n")
            f.write("END_POPLAN\n")
                
        f.close()

    @coroutine
    def run(self):
        if self.failure_simulated:
            self.handle_task_failure()
            return

        self.plan_state = None
        
        try:
            to = timeout(WAIT_FOR_CONSISTENCY_TIMEOUT)
            while not self.state.consistent:
                self.update_status(TaskStateEnum.WAITING_FOR_BELIEF, TaskStateInfoEnum.WAITING_FOR_CONSISTENT_STATE)
                yield to.next()
            yield TO_OK
        except TimedOut:
            log.warning("Wait for consistent state timed out. Plan failed.")
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.NO_CONSISTENT_STATE)
            return
        
        self.update_status(TaskStateEnum.PROCESSING, TaskStateInfoEnum.PLANNING_CP)
        self.plan_state = self.state
        self.cp_task.replan()
        log.debug("Planning done: %.2f sec", global_vars.get_time())
        self.plan_log.append(planner_log.PlanEntry(self.cp_task.get_plan()))

        if self.cp_task.get_plan() is None:
            self.plan_history.append(None)
            self.plan_state_history.append((None, None, self.plan_state))
            self.write_history()
        else:
            new_po_plan = self.make_cast_poplan(self.cp_task.get_plan(), is_completed=False)
            self.component.deliver_po_plan(self, new_po_plan)
            
        self.process_cp_plan()
        log.debug("Plan processing done: %.2f sec", global_vars.get_time())

    @coroutine
    def retry(self):
        try:
            to = timeout(WAIT_FOR_CONSISTENCY_TIMEOUT)
            while not self.state.consistent:
                self.update_status(TaskStateEnum.WAITING_FOR_BELIEF, TaskStateInfoEnum.WAITING_FOR_CONSISTENT_STATE)
                yield to.next()
            yield TO_OK
        except TimedOut:
            log.warning("Wait for consistent state timed out. Plan failed.")
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.NO_CONSISTENT_STATE)
            return

        plan = self.cp_task.get_plan()

        self.cp_task.mark_changed()
        self.update_status(TaskStateEnum.PROCESSING, TaskStateInfoEnum.PLANNING_CP)
        self.cp_task.replan()
        if self.cp_task.planning_status == PlanningStatusEnum.WAITING:
            self.cp_task.set_plan(None)
            self.cp_task.mark_changed()
            self.cp_task.replan()
            

        if self.cp_task.get_plan() != plan:
            problem_fn = abspath(join(self.component.get_path(), "problem%d-%d.pddl" % (self.id, len(self.plan_history)+1)))
            self.plan_log.append(planner_log.PlanEntry(self.cp_task.get_plan()))
            self.write_cp_problem(problem_fn)
            #way may reach this point without ever having planned before. Check that case
            if self.plan_state is not None:
                self.plan_history.append(plan)
                po_plan = self.make_cast_poplan(plan, is_completed=True)
                self.plan_state_history.append((plan, po_plan, self.plan_state))
                self.write_history()
                self.component.deliver_po_plan(self, po_plan)

            new_po_plan = self.make_cast_poplan(self.cp_task.get_plan(), is_completed=False)
            self.component.deliver_po_plan(self, new_po_plan)

            self.plan_state = self.state
        
        self.process_cp_plan()
            
    def handle_task_failure(self):
        from standalone import merge_plans
        log.info("Trying to find explanations for failure of task %d", self.id)
        self.plan_log.append(planner_log.TaskStatusEntry(TaskStateEnum.FAILED))
        
        plan = self.cp_task.get_plan()
        if not self.failure_simulated:
            self.plan_history.append(plan)
            po_plan = self.make_cast_poplan(plan, is_completed=True)
            self.plan_state_history.append((plan, po_plan, self.plan_state))
            self.update_info_status(TaskStateInfoEnum.EXPLANATIONS_PENDING) 
            self.write_history()
            self.component.deliver_po_plan(self, po_plan)
        
            self.component.verbalise("Oh, plan execution failed unexpectedly.  I'm searching for an explanation now.")
            time.sleep(5)

        print self.init_state.state
        print "==============================\n"*3
        merged_plan, init_state, final_state = merge_plans.merge_plans(self.plan_history, self.init_state.state, self.state.state)
        # last_plan = self.plan_history[-1].topological_sort()
        # last_plan = merged_plan.topological_sort()
        # endstate = self.state.state.copy()
        # for a in last_plan:
        #     if a.status == plans.ActionStatusEnum.EXECUTED and not a.is_virtual():
        #         for f in a.effects:
        #             endstate.set(f)
        if self.expl_rules_fn and len(merged_plan) > 2:
            result, expl_plan = explanations.handle_failure(merged_plan, init_state.problem, init_state, final_state, self.expl_rules_fn, self.cp_task, self.component)
            if result:
                self.update_info_status(TaskStateInfoEnum.EXPLANATIONS_FOUND) 
                facts = reduce(lambda x,y:x|y.effects, result, set())
                log.debug("Raw explanations: %s", ", ".join(str(f) for f in facts))
                beliefs = list(self.facts_to_belief(facts))

                expl_poplan = self.make_cast_poplan(expl_plan, is_completed=True)
                for s in self.poplan_to_string(expl_poplan):
                    print s
                    
                self.component.deliver_hypotheses(self, beliefs)
            else:
                log.info("Found no explanations.")
                self.update_info_status(TaskStateInfoEnum.EXPLANATIONS_NOT_FOUND) 
                

    def facts_to_belief(self, facts):
        VIRTUAL_OBJ_RE = re.compile("([a-z]+)([0-9]+)")
        cast_obj_re = re.compile("[a-z]+__?[0-9a-z]__?[0-9a-z]")
        obj_to_feature = defaultdict(set)
        obj_to_relation = defaultdict(set)
        for f in facts:
            if f.svar.modality == pddl.mapl.commit:
                assert f.value == pddl.TRUE
                f = pddl.state.Fact(f.svar.nonmodal(), f.svar.modal_args[0])
            if len(f.svar.args) == 1:
                obj_to_feature[f.svar.args[0]].add(f)
            elif len(f.svar.args) > 1:
                for a in f.svar.args:
                    obj_to_relation[a].add(f)

        def is_virtual_object(obj):
            match = VIRTUAL_OBJ_RE.search(obj.name)
            return (match and match.group(1) in self.domain.types)


        about_func = pddl.Function("about", [pddl.Parameter("?o", pddl.t_object)], pddl.t_object)

        new_objects = set()

        for o in obj_to_feature.keys():
            if is_virtual_object(o):
                for f in self.state.state.iterfacts():
                    if f.svar.modality is not None or f.svar.function.name == "is-virtual":
                        continue
                    if o in f.svar.args:
                        if len(f.svar.args) == 1:
                            obj_to_feature[o].add(f)
                        else:
                            obj_to_relation[o].add(f)

                new_objects.add(o)
                wma = self.component.newAddress()
                self.state.obj_to_castname[o] = wma.id
                self.state.address_dict[wma.id] = wma
            else:
                about_fact =  pddl.state.Fact(pddl.state.StateVariable(about_func, [o]), o)
                obj_to_feature[o].add(about_fact)
                

        beliefs = []
        log.info("\nexplanations:")
        for o, vals in obj_to_feature.iteritems():
            obj_facts = itertools.chain(vals, obj_to_relation.get(o, []))
            log.info("  %s: %s", str(o), ", ".join(str(f) for f in obj_facts))
            features = sum((self.state.make_features(f) for f in vals), [])
            # bel = self.state.make_belief(features, HypotheticalBelief)
            bel = self.state.make_belief(features, HypotheticalBelief)
            bel.type = o.type.name
            if o in new_objects:
                # use the previously generated id for new objects
                bel.id = self.state.obj_to_castname[o]
            else:
                bel.id = self.component.newAddress().id
            yield bel
            
        for o, vals in obj_to_relation.iteritems():
            if o not in obj_to_feature:
                log.info("  %s: %s", str(o), ", ".join(str(f) for f in vals))

            for f in vals:
                bel = self.state.make_belief(self.state.make_features(f), HypotheticalBelief)
                bel.id = self.component.newAddress().id
                yield bel


    def process_cp_plan(self):
        plan = self.get_plan()
        
        if plan is None:
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.PLANNING_FAILURE)
            return

        if FAKE_ACTION_FAILURE is not None:
            ordered_plan = plan.topological_sort()
            for pnode in ordered_plan:
                if pnode.action.name == FAKE_ACTION_FAILURE:
                    pnode.status = plans.ActionStatusEnum.FAILED
                    break
                else:
                    pnode.status = plans.ActionStatusEnum.EXECUTED
            self.plan_history.append(plan)
            self.handle_task_failure()
            return
        

        total_prob = reduce(lambda x,y: x*y, (n.prob for n in plan.nodes_iter()), 1.0)
        if total_prob < self.component.min_p:
            log.warning("total probability %.4f below threshold %.4f. Task failed", total_prob, self.component.min_p)
            # self.plan_history.append(plan)
            self.cp_task.set_plan(None)
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.PLANNING_FAILURE)
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
            log.debug("creating dt task")
            # self.dt_task = dt_problem.DTProblem(plan, self.domain)
            self.dt_task = dt_problem.DTProblem(plan, self.state.pnodes, self.qgraph, self.fail_count, self.state.get_prob_functions(), self.relevant_facts, self.domain)
            # self.dt_task.initialize(self.state.prob_state)

            for pnode in plan.nodes_iter():
                if pnode.is_virtual():
                    pnode.status = plans.ActionStatusEnum.EXECUTED
            
            if self.dt_planning_active():
                log.info("starting dt task")
                self.plan_log.append(planner_log.DTEntry(enabled=True))
                self.dt_task.initialize(self.state.prob_state)
                self.update_status(TaskStateEnum.WAITING_FOR_DT, TaskStateInfoEnum.PLANNING_DT)
                for pnode in self.dt_task.subplan_actions:
                    pnode.status = plans.ActionStatusEnum.IN_PROGRESS
                self.component.start_dt_planning(self)
                return
            
        log.debug("Current plan:\n%s", plan)

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
        self.dispatch_actions(exec_plan, plan)

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
            self.plan_log.append(planner_log.DTEntry(cancelled=True))
            self.dt_done()
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

    @coroutine
    def monitor_dt(self):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION)
        
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            # log.debug(" examining action %s", str(pnode))
            if any(fact not in self.state.state for fact in pnode.effects):
                sat = False
                break
            # else:
            #     log.debug("effects are satisfied: %s", ", ".join(map(str, pnode.effects)))
        if sat:
            log.info("dt planner reached its goal.")
            self.plan_log.append(planner_log.DTEntry(enabled=False))
            self.dt_done()
            return

        dt_pnode = self.dt_task.dt_plan[-1]

        if self.dt_task.replanning_neccessary(self.state.prob_state):
            log.info("DT task requires replanning")
            self.component.cancel_dt_session(self)
            self.dt_task.recompute_problem(self.state.prob_state)
            self.update_status(TaskStateEnum.WAITING_FOR_DT, TaskStateInfoEnum.PLANNING_DT)
            self.component.start_dt_planning(self)

        def get_observations():
            result = []
            for fact in self.state.convert_percepts(self.percepts):
                if fact.svar.modality == pddl.mapl.attributed:
                    svar = fact.svar.nonmodal()
                    value = fact.svar.modal_args[1]
                else:
                    svar, value = fact
                pred = "observed-%s" % svar.function.name
                obs = Planner.Observation(pred, [a.name for a in svar.args] + [value.name])
                result.append(obs)
                log.info("%d: delivered observation (%s %s)", self.id, obs.predicate, " ".join(a for a in obs.arguments))
            return result

        if not self.dt_task.observation_expected(dt_pnode.action):
            log.debug("No observations expected from %s", str(self.dt_task.dt_plan[-1]))
            observations = [Planner.Observation("null", [])]
        else:
            try:
                to = timeout(WAIT_FOR_ACTION_TIMEOUT/2)
                while True:
                    observations = get_observations()
                    if not observations:
                        log.info("Waiting for observations from %s", str(self.dt_task.dt_plan[-1]))
                        self.update_status(TaskStateEnum.WAITING_FOR_BELIEF, TaskStateInfoEnum.WAITING_FOR_OBSERVATION)
                        yield to.next()
                    else:
                        yield TO_OK
                        break
            except TimedOut:
                log.info("Got no observations from %s", str(self.dt_task.dt_plan[-1]))
                observations = [Planner.Observation("null", [])]
                        
        log.debug("delivered observations")
        self.update_status(TaskStateEnum.WAITING_FOR_DT, TaskStateInfoEnum.PLANNING_DT)
        self.component.getDT().deliverObservation(self.dt_id, observations)

    @coroutine
    def monitor_cp(self):
        assert self.internal_state in (TaskStateEnum.PROCESSING, TaskStateEnum.WAITING_FOR_ACTION)
        
        # if self.dt_planning_active():
        #     self.process_cp_plan()
        #     return

        self.update_status(TaskStateEnum.PROCESSING, TaskStateInfoEnum.PLANNING_CP)

        problem_fn = abspath(join(self.component.get_path(), "problem%d.pddl" % (self.id)))
        self.write_cp_problem(problem_fn)

        
        try:
            to = timeout(WAIT_FOR_CONSISTENCY_TIMEOUT)
            while not self.state.consistent:
                self.update_status(TaskStateEnum.WAITING_FOR_BELIEF, TaskStateInfoEnum.WAITING_FOR_CONSISTENT_STATE)
                yield to.next()
            yield TO_OK
        except TimedOut:
            log.warning("Wait for consistent state timed out. Plan failed.")
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.NO_CONSISTENT_STATE)
            return

        plan = self.cp_task.get_plan()
        
        self.step += 1

        try:
            to = timeout(WAIT_FOR_ACTION_TIMEOUT)
            while True:
                self.cp_task.replan()
                log.debug("Continual planning step done: %.2f sec", global_vars.get_time())
                
                if self.cp_task.get_plan() != plan:
                    problem_fn = abspath(join(self.component.get_path(), "problem%d-%d.pddl" % (self.id, len(self.plan_history)+1)))
                    self.write_cp_problem(problem_fn)
                    self.plan_log.append(planner_log.PlanEntry(self.cp_task.get_plan()))
                    self.plan_history.append(plan)
                    po_plan = self.make_cast_poplan(plan, is_completed=True)
                    self.plan_state_history.append((plan, po_plan, self.plan_state))
                    self.write_history()
                    self.plan_state = self.state
                    
                    new_po_plan = self.make_cast_poplan(self.cp_task.get_plan(), is_completed=False)
                    self.component.deliver_po_plan(self, po_plan)
                    self.component.deliver_po_plan(self, new_po_plan)
                    
                if self.cp_task.planning_status == PlanningStatusEnum.WAITING:
                    log.info("Waiting for effects of %s to appear", str(self.cp_task.pending_action))
                    self.update_status(TaskStateEnum.WAITING_FOR_BELIEF, TaskStateInfoEnum.WAITING_FOR_EFFECT)
                    yield to.next()
                else:
                    yield TO_OK
                    break
                
        except TimedOut:
            log.info("Wait for %s timed out. Plan failed.", str(self.cp_task.pending_action))
            # self.plan_history.append(plan)
            # self.cp_task.set_plan(None, update_status=True)
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.EXECUTION_FAILURE)
            return
        
        self.process_cp_plan()

    @coroutine
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
            
        self.get_plan().execution_position = last_dt_action
        self.update_status(TaskStateEnum.PROCESSING, TaskStateInfoEnum.PLANNING_CP)
        self.plan_history.append(self.dt_task)

        # import debug
        # debug.set_trace()

        self.cp_task.mark_changed()

        # use the timeout mechanism to escape the DT thread
        log.debug("returning to main loop...")
        try:
            yield 0
            yield TO_OK
        except TimedOut:
            pass

        self.component.cancel_dt_session(self)
        log.debug("Continuing sequential session")
        self.monitor_cp()
        

    @coroutine
    def action_delivered(self, action):
        assert self.internal_state == TaskStateEnum.WAITING_FOR_DT
        
        log.debug("raw action: (%s %s)", action.name, " ".join(action.arguments))
        args = [self.cp_task.mapltask[a] for a in action.arguments]
        pddl_action = self.dt_task.dtdomain.get_action(action.name)
        log.debug("got action from DT: (%s %s)", action.name, " ".join(action.arguments))
        #log.debug("state is: %s", self.cp_task.get_state())

        if self.dt_task.is_goal_action(pddl_action.name):
            log.info("Goal action recieved. DT task completed")
            self.plan_log.append(planner_log.DTEntry(enabled=False))
            self.dt_done()
            return

        pnode = None
        try:
            to = timeout(WAIT_FOR_ACTION_TIMEOUT)
            while True:
                #TODO: using the last CP state might be problematic
                state = self.cp_task.get_state().copy()
                try:
                    pnode = plan_postprocess.getRWDescription(pddl_action, args, state, 1)
                    yield TO_OK
                    break
                except:
                    log.info("Action (%s %s) not executable. Waiting for action effects from previous action...", pddl_action.name, " ".join(action.arguments))
                    yield to.next()

        except TimedOut:
            self.plan_log.append(planner_log.DTEntry(cancelled=True))
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
                self.plan_log.append(planner_log.ActionEntry(pnode))

            self.plan_history.append(self.dt_task)
            # self.cp_task.set_plan(None, update_status=True)
            self.component.cancel_dt_session(self)
            self.update_status(TaskStateEnum.FAILED, TaskStateInfoEnum.EXECUTION_FAILURE)
            return
        
        self.percepts = []
        self.dt_task.dt_plan.append(pnode)
        self.dispatch_actions([pnode])

    def dispatch_actions(self, nodes, plan=None):
        actions = {}
        # create CAST actions for each plan node
        all_nodes = plan.nodes_iter() if plan else nodes
        for pnode in all_nodes:
            uargs = [self.state.featvalue_from_object(a) for a in pnode.args]
            full_uargs = [self.state.featvalue_from_object(a) for a in pnode.full_args]
            fullname = "%s %s" % (pnode.action.name, " ".join(fval_to_str(a) for a in uargs))
            actions[pnode] = Planner.Action(self.id, pnode.action.name, uargs, full_uargs, fullname, float(pnode.cost), Planner.Completion.PENDING)
        outplan = [actions[n] for n in nodes]

        if outplan:
            log.info("First action: %s == %s", str(nodes[0]), outplan[0].fullName)
            nodes[0].status = plans.ActionStatusEnum.IN_PROGRESS
            self.plan_log.append(planner_log.ActionEntry(nodes[0]))
            self.update_status(TaskStateEnum.WAITING_FOR_ACTION, TaskStateInfoEnum.WAITING_FOR_ACTION)
        else:
            log.info("Plan is empty")
            self.plan_log.append(planner_log.TaskStatusEntry(TaskStateEnum.COMPLETED))
            self.update_status(TaskStateEnum.COMPLETED, TaskStateInfoEnum.COMPLETED)

        self.component.deliver_plan(self, outplan)

    def make_cast_poplan(self, plan, is_completed):
        status = Planner.POPlanStatus.FINISHED if is_completed else Planner.POPlanStatus.RUNNING
        
        if plan:
            ordered_nodes = list(plan.topological_sort())
            indexes = dict((n,i) for i,n in enumerate(ordered_nodes))
            po_actions = []
            for pnode in ordered_nodes:
                uargs = [self.state.featvalue_from_object(a) for a in pnode.args]
                full_uargs = [self.state.featvalue_from_object(a) for a in pnode.full_args]
                fullname = "%s %s" % (pnode.action.name, " ".join(fval_to_str(a) for a in uargs))
                action_status = action_status_dict[pnode.status]
                po_actions.append(Planner.Action(self.id, pnode.action.name, uargs, full_uargs, fullname, float(pnode.cost), action_status))

            #Build partially ordered plan in CAST representation
            po_links = []
            for n1,n2, data in plan.edges_iter(data=True):
                if data['type'] == 'depends':
                    typ = Planner.LinkType.DEPENDS
                elif data['type'] == 'prevent_threat':
                    typ = Planner.LinkType.THREATENS
                elif data['type'] == 'unexpected':
                    typ = Planner.LinkType.UNEXPECTED
                elif data['type'] == 'repaired':
                    typ = Planner.LinkType.FIXED
                elif data['type'] == 'explanation':
                    typ = Planner.LinkType.EXPLANATION
                else:
                    continue

                svar = data['svar']
                svar_args = [self.state.featvalue_from_object(a) for a in svar.args]
                svar_modal_args = [self.state.featvalue_from_object(a) for a in svar.modal_args]
                value = self.state.featvalue_from_object(data['val'])
                # if typ == Planner.LinkType.UNEXPECTED:
                #     print svar, data['val']
                #     print svar_args, svar_modal_args, value
                fact = Planner.Fact(svar.function.name, svar_args, svar.modality.name if svar.modality else "", svar_modal_args, value)
                link = Planner.Link(typ, indexes[n1], indexes[n2], fact)
                po_links.append(link)

            po_plan = Planner.POPlan(self.id, status, po_actions, po_links)
        else:
            po_plan = Planner.POPlan(self.id, status, [], [])

        return po_plan
        

    def poplan_to_string(self, po_plan):
        if not po_plan or not po_plan.actions:
            yield "actions:"
            return
            
        def print_featvalue(fv):
            if isinstance(fv, logicalcontent.BooleanFormula):
                return "true" if fv.val else "false"
            elif isinstance(fv, logicalcontent.FloatFormula):
                return str(fv.val)
            elif isinstance(fv, logicalcontent.PointerFormula):
                return "%s@%s" % (fv.pointer.id, fv.pointer.subarchitecture)
            else:
                return str(fv.prop)
            
        yield "actions:"
        for i, a in enumerate(po_plan.actions):
            yield "%d: %s %s %s" % (i, a.status, a.name, " ".join(print_featvalue(arg) for arg in a.allArguments))
        yield "links:"
        for link in po_plan.links:
            fact = "%s %s" % (link.reason.name, " ".join(map(print_featvalue, link.reason.arguments)))
            if link.reason.modality:
                fact = "%s MODALITY: %s %s" % (fact, link.reason.modality, " ".join(map(print_featvalue, link.reason.modalArguments)))
            fact = "%s VALUE: %s" % (fact, print_featvalue(link.reason.value))
            yield "%d %d %s %s" % (link.src, link.dest, link.type, fact)
        
    def action_feedback(self, finished_actions, failed_actions):
        for pnode in itertools.chain(finished_actions, failed_actions):
            self.plan_log.append(planner_log.ActionEntry(pnode))

        diffstate = compute_state_updates(self.cp_task.get_state(), finished_actions, failed_actions)
        for fact in diffstate.iterfacts():
            self.cp_task.get_state().set(fact)
        #TODO: create new state?
        beliefs = self.state.update_beliefs(diffstate)
        self.component.update_cast_beliefs(beliefs)

    def compute_solution_likelihood(self):
        from standalone import relaxed_exploration
        goal = self.cp_task.mapltask.goal
        domain = self.cp_task.mapltask.domain
        
        goal_action = pddl.Action("goal_action", [], goal, None, self.cp_task.mapltask)
        # det_state = self.state.determinized_state(sw_conf.rejection_ratio, sw_conf.known_threshold)
        actions = [a for a in domain.actions if not a.name.startswith("__commit")]

        # t0 = time.time()
        # if not self.unary_ops:
        #     self.unary_ops, applicable_ops = relaxed_exploration.forward_instantiate(domain.actions, set(), self.state.state, domain, [(goal_action, {})])
        # else:
        #     applicable_ops = relaxed_exploration.initialize_ops(self.unary_ops, self.state.state)
        # print "total time for preparation: %.2f" % (time.time()-t0)
        # t0 = time.time()
        
        # rel, p = relaxed_exploration.prob_explore(self.unary_ops, applicable_ops, self.state.state, [(goal_action, [])], self.state.prob_state)
        # print "total time for exploration: %.2f" % (time.time()-t0)

        self.relevant_facts = None
        self.solution_prob = 1.0
        
        return self.solution_prob
        
    def update_state(self, beliefs):
        assert self.internal_state in (TaskStateEnum.WAITING_FOR_ACTION, TaskStateEnum.WAITING_FOR_BELIEF, TaskStateEnum.FAILED)
        
        import fake_cast_state
        if isinstance(self.state, fake_cast_state.FakeCASTState):
            return True

        oldstate = self.state
        self.state = cast_state.CASTState(beliefs, self.domain, oldstate, component=self.component, consistency_cond=self.consistency_cond)
        new_cp_problem, new_cp_domain, self.goaldict = self.state.to_problem(self.slice_goals, deterministic=True)

        self.state.print_state_difference(oldstate)
        
        for g in self.slice_goals:
            if g.importance == -1 and g.goalString not in self.goaldict:
                log.info("Hard goal %s cannot be parsed; planning failed" % g.goalString)
                return False

        self.compute_solution_likelihood()
        
        self.cp_task.mapltask = new_cp_problem
        self.cp_task.set_state(self.state.state)
        log.debug("State update done: %.2f sec", global_vars.get_time())
        self.plan_log.append(planner_log.StateEntry(self.state, self.slice_goals))
        
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
