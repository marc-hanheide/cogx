import sys, itertools, time

import config
import assertions#, macros
import pddl
from  pddl import state
import statistics

from utils import Enum

log = config.logger("planner")

PlanningStatusEnum = Enum("TASK_CHANGED", "RUNNING", "PLAN_AVAILABLE", "PLANNING_FAILURE", "INTERRUPTED", "WAITING")    

statistics_defaults = dict(
    deliberate_replans=0,
    )


class Task(object):
    """
    """

    def __init__(self, taskID=0, mapltask = None, add_assertions = False):
        """Initialise public and private fields."""
        # public
        self.taskID = taskID
        self.statistics = statistics.Statistics(statistics_defaults)
        self._mapltask = mapltask
        self._mapldomain = None
        self._action_blacklist = None
        self._action_whitelist = None
        self._plan = None
        self.planner = None
        self.pending_action = None
        self.wait_for_effects = True
        #for dt testing only
        self.dt_calls = 0
        self.dt_actions = 0
        self.dt_orig_id = 0
        # private
        self._state = None
        
        if mapltask:
            self._mapldomain = mapltask.domain
            self.create_initial_state()
            if add_assertions:
                self.add_assertions()
                
        self.planning_status = PlanningStatusEnum.TASK_CHANGED

    def __get_mapltask(self):
        if self.is_dirty():
            ann = pddl.translators.Translator.get_annotations(self._mapltask)
            new_init = [ f.as_literal(useEqual=True) for f in self.get_state().iterfacts() ]
            self._mapltask = pddl.Problem(self._mapltask.name, self._mapltask.objects, new_init, self._mapltask.goal, self._mapldomain, self._mapltask.optimization, self._mapltask.opt_func)
            self._mapltask.annotations = ann
            
        return self._mapltask

    def __set_mapltask(self, mapltask):
        self._mapltask = mapltask
        self._mapldomain = mapltask.domain

    mapltask = property(__get_mapltask, __set_mapltask)

    @property
    def mapldomain(self):
        return self._mapldomain

    def add_assertions(self):
        new_assertions = []
        for a in self._mapldomain.actions:
            ast = assertions.to_assertion(a, self._mapldomain)
            if ast:
                try:
                    self._mapldomain.get_action(ast.name)
                except:
                    new_assertions.append(ast)

        self._mapldomain.actions += new_assertions
        self._mapldomain.name2action = None

    def create_initial_state(self):
        s = state.State([], self._mapltask)
        for i in self._mapltask.init:
            #determinise probabilistic init conditions
            if isinstance(i, pddl.effects.ProbabilisticEffect):
                pass
                # for p, eff in i.effects:
                #     facts = s.get_effect_facts(eff)
                #     for svar, value in facts.iteritems():
                #         if not isinstance(svar, pddl.Predicate) and svar.modality is None:
                #             if svar in s:
                #                 del s[svar]
                #             id_var = svar.as_modality(mapl.i_indomain, [value])
                #             s[id_var] = pddl.TRUE
            else:
                s.set(state.Fact.from_literal(i))
        self._state = s

    def mark_changed(self):
        self.planning_status =  PlanningStatusEnum.TASK_CHANGED

    def is_dirty(self):
        """Check if task has been modified so that the plan is possibly outdated."""
        return self.planning_status == PlanningStatusEnum.TASK_CHANGED

    def get_state(self):
        return self._state

    def _change_task(self, field, new_val, update_status=True):
        """
        Changes a field in the task and marks the task as changed
        (which might trigger replanning) if necessary
        """
        old_val = self.__dict__[field]
        self.__dict__[field] = new_val
        if update_status and old_val != new_val:
            self.planning_status = PlanningStatusEnum.TASK_CHANGED

    def set_state(self, new_val, update_status=True):
        self._change_task("_state", new_val, update_status) 
            
    def get_goal(self):
        return self._mapltask.goal

#    def set_goal(self, update_status=True):
#        self._change_task("_goal", new_val, update_status) 

    def get_plan(self):
        return self._plan

    def set_planning_status(self, status):
        self.planning_status = status

    def set_plan(self, plan, update_status=True):
        self._plan = plan
        if update_status:
            if plan is None:
                self.planning_status = PlanningStatusEnum.PLANNING_FAILURE
            else:
                self.planning_status = PlanningStatusEnum.PLAN_AVAILABLE

    def replan(self):
        """If the task is registered with a planner, the plan will be updated,
        but only if the task has been modified and  if change detection is activated."""
        if self.is_dirty() and self.planner:
            self.planner.continual_planning(self)
        return self.get_plan()

    # def problem_str(self, writer_class):
    #     w = writer_class()
    #     return "\n".join(w.write_problem(self.mapltask))

    # def domain_str(self, writer_class):
    #     w = writer_class()
    #     return "\n".join(w.write_domain(self._mapldomain))
    
    def load_mapl_domain(self, domain_file):
        log.info("Loading MAPL domain %s.", domain_file)
        self._mapldomain = pddl.load_domain(domain_file)
        
        if "partial-observability" in self._mapldomain.requirements:
            self.dtpddl_domain = self._mapldomain
            self._mapldomain = pddl.dtpddl.DT2MAPLCompiler().translate(self._mapldomain)
        
        
    def load_mapl_problem(self, task_file, agent_name=None):
        log.info("Loading MAPL problem %s.", task_file)
        self.mapltask = pddl.load_problem(task_file, self._mapldomain)
        self.create_initial_state()

        self._agent_name = agent_name

    def parse_mapl_problem(self, problem_str, agent_name=None):
        self._mapltask = pddl.parse_problem(problem_str, self._mapldomain)
        self.create_initial_state()

        self._agent_name = agent_name

    def load_mapl_task(self, task_file, domain_file, agent_name=None):
        self.load_mapl_domain(domain_file)
        self.load_mapl_problem(task_file, agent_name)


class PDDLOutput(object):
    def __init__(self, writer=None, compiler=None):
        self.compiler = compiler
        self.supported = None
        if writer:
            self.writer = writer
        else:
            self.writer = pddl.writer.Writer()

    def translate(self, problem, domain=None):
        if not self.compiler and not self.supported:
            if not domain:
                domain = problem.domain
            return domain, problem

        compiler = self.compiler
        if self.supported:
            t = problem.domain.compile_to(self.supported)
            if t.depends and self.compiler:
                compiler = pddl.translators.ChainingTranslator(t, self.compiler)
            elif t.depends:
                compiler = t
        
        t0 = time.time()
        if domain is not None and domain != problem.domain:
            tr_dom = compiler.translate(domain)
            tr_prob = compiler.translate(problem)
        else:
            tr_prob = compiler.translate(problem)
            tr_dom = tr_prob.domain
        log.debug("total time for translation: %f", time.time()-t0)
        return tr_dom, tr_prob

    def print_problem(self, problem, domain=None, fd=None):
        tr_dom, tr_prob = self.translate(problem, domain)
        if not fd:
            fd = sys.stdout
            
        prob_str = self.writer.write_problem(tr_prob)
        for l in prob_str:
            print >> fd, l
    
    def write(self, problem, domain=None, domain_fn=None, problem_fn=None):
        tr_dom, tr_prob = self.translate(problem, domain)
        dom_str = self.writer.write_domain(tr_dom)
        prob_str = self.writer.write_problem(tr_prob)

        if domain_fn is not None:
            f = open(domain_fn, "w")
            for l in dom_str:
                f.write(l)
                f.write("\n")
            f.close()
        if problem_fn is not None:
            f = open(problem_fn, "w")
            for l in prob_str:
                f.write(l)
                f.write("\n")
            f.close()

        return dom_str, prob_str

adl_support = ["strips", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "adl", "derived-predicated"]
    
class ADLOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = pddl.translators.ADLCompiler()
        self.supported = adl_support
    
class TFDOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = TemporalTranslator()
        self.supported = adl_support + ['durative-actions', 'fluents']

class FDOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = pddl.translators.ADLCompiler()
        self.supported = adl_support + ['action-costs']
        
    def write(self, problem, domain=None, domain_fn=None, problem_fn=None, mutex_fn=None):
        tr_dom, tr_prob = self.translate(problem, domain)
        tr_dom.requirements.discard('numeric-fluents')

        mutex_groups = self.get_mutex_groups(tr_prob, problem)
        
        dom_str = self.writer.write_domain(tr_dom)
        prob_str = self.writer.write_problem(tr_prob)
        mutex_str = self.write_mutex(mutex_groups)

        if domain_fn is not None:
            f = open(domain_fn, "w")
            for l in dom_str:
                f.write(l)
                f.write("\n")
            f.close()
        if problem_fn is not None:
            f = open(problem_fn, "w")
            for l in prob_str:
                f.write(l)
                f.write("\n")
            f.close()
        if mutex_fn is not None:
            f = open(mutex_fn, "w")
            for l in mutex_str:
                f.write(l)
                f.write("\n")
            f.close()

        return dom_str, prob_str, mutex_str
    
    def get_mutex_groups(self, prob, oldprob):
        groups = []

        build_modal_groups = ('mapl' in oldprob.domain.requirements)

        for function in oldprob.functions:
            pred = prob.predicates.get(function.name, function.args+[function.type])
            if build_modal_groups:
                commit_pred = prob.predicates.get("%s-%s" % (pddl.mapl.commit.name, function.name), function.args+[function.type])
                
            fluents_combinations = state.product(*(prob.get_all_objects(a.type) for a in function.args))
            for c in fluents_combinations:
                group = []
                commit_group = []
                for v in prob.get_all_objects(function.type):
                    #print pddl.Literal(pred, c+[v]).pddl_str()
                    group.append(pddl.Literal(pred, itertools.chain(c, [v])))
                    if build_modal_groups and commit_pred:
                        commit_group.append(pddl.Literal(commit_pred, itertools.chain(c, [v])))
                    
                groups.append(group)
                if build_modal_groups:
                    groups.append(commit_group)
        
        return groups
    
    def write_mutex(self, mutex_groups):
        groups = []
        for g in mutex_groups:
            strings = []
            for atom in g:
                strings.append(self.writer.write_literal(atom))
            groups += self.writer.section("", strings)
        return self.writer.section("mutex", groups)

        

class TemporalTranslator(pddl.translators.Translator):
    def __init__(self, copy=True):
        self.depends = [pddl.translators.ModalPredicateCompiler(remove_replan=True), pddl.translators.PreferenceCompiler()]
        self.set_copy(copy)
        self.lock_pred = pddl.Predicate("locked", [])
        self.ended_pred = pddl.Predicate("ended", [])
        self.do_locking = False

    def translate_action(self, action, domain=None):
        if isinstance(action, pddl.durative.DurativeAction):
            a2 = pddl.translators.Translator.translate_action(self, action, domain)
        else:
            total_costs = action.get_total_cost()
            if total_costs == 0:
                dc = pddl.durative.DurationConstraint(pddl.Term(0.001))
            elif total_costs:
                dc = pddl.durative.DurationConstraint(total_costs)
            else:
                dc = pddl.durative.DurationConstraint(pddl.Term(1))

            args = [pddl.Parameter(p.name, p.type) for p in action.args]
            a2 = pddl.durative.DurativeAction(action.name, args, [dc], None, None, domain)

        def condition_visitor(cond, results):
            if isinstance(cond, pddl.durative.TimedCondition):
                return cond, True
            if isinstance(cond, pddl.Literal) and cond.predicate == pddl.builtin.change:
                return cond, True
            if all(timed == False for r, timed in results):
                return cond, False
            if all(timed == True for r, timed in results):
                return cond, True
            timed_parts = [r for r,timed in results if timed == True]
            untimed_parts = [r for r,timed in results if timed == False]
            timed_cond = pddl.durative.TimedCondition("start", cond.copy(new_parts=untimed_parts), cond.get_scope())
            timed_parts.append(timed_cond)
            return cond.copy(new_parts=timed_parts), True

        tct = pddl.Term(pddl.builtin.total_cost,[])
        @pddl.visitors.copy
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.SimpleEffect):
                if eff.predicate == pddl.builtin.increase and eff.args[0] == tct:
                    return False
                if eff.predicate == pddl.builtin.change:
                    return eff
                return pddl.durative.TimedEffect(eff.predicate, eff.args[:], "end", negated=eff.negated)
            if isinstance(eff, pddl.ConditionalEffect):
                eff2 = pddl.ConditionalEffect(None, results[0])
                cond, is_timed = eff.condition.visit(condition_visitor)
                if not is_timed:
                    cond = pddl.durative.TimedCondition("start", cond, a2)
                eff2.condition = cond
                return eff2

        a2.precondition, is_timed = pddl.visitors.visit(action.precondition, condition_visitor, default=(None, True))
        if not is_timed:
            a2.precondition = pddl.durative.TimedCondition("start", a2.precondition, a2)
        a2.effect = pddl.visitors.visit(action.effect, effect_visitor)
        return a2
        # 

        # affected_vars = set()
        # cond_effects = []
        
        # @pddl.visitors.copy
        # def effect_visitor(eff, results):
        #     if isinstance(eff, pddl.SimpleEffect):
        #         if eff.predicate == pddl.builtin.increase and eff.args[0] == tct:
        #             return False
        #         if eff.predicate == pddl.assign:
        #             affected_vars.add((eff.args[0].function, ) + tuple(eff.args[0].args))
        #             return pddl.SimpleEffect(pddl.durative.change, eff.args[:])
        #         if eff.predicate == pddl.builtin.num_assign:
        #             affected_vars.add((eff.args[0].function, ) + tuple(eff.args[0].args))
        #             return pddl.SimpleEffect(pddl.durative.num_change, eff.args[:])
        #         affected_vars.add((eff.predicate, ) + tuple(eff.args))
        #         return pddl.durative.TimedEffect(eff.predicate, eff.args[:], "end", negated=eff.negated)
        #     if isinstance(eff, pddl.ConditionalEffect):
        #         eff2 = pddl.ConditionalEffect(None, results[0])
        #         eff2.condition = pddl.durative.TimedCondition("all", eff.condition.copy())
        #         cond_effects.append(eff2)
        #         return eff2

        # if isinstance(action, pddl.durative.DurativeAction):
        #     return action.copy(newdomain=domain)

        # total_costs = action.get_total_cost()
        # if total_costs == 0:
        #     dc = pddl.durative.DurationConstraint(pddl.Term(0.001))
        # elif total_costs:
        #     dc = pddl.durative.DurationConstraint(total_costs)
        # else:
        #     dc = pddl.durative.DurationConstraint(pddl.Term(1))
            
        # args = [pddl.Parameter(p.name, p.type) for p in action.args]
        # a2 = pddl.durative.DurativeAction(action.name, args, [dc], None, None, domain)

        # lock_cond = pddl.durative.TimedCondition("start", pddl.LiteralCondition(self.lock_pred, [], a2, negated=True))
        # ended_cond = pddl.durative.TimedCondition("all", pddl.LiteralCondition(self.ended_pred, [], a2, negated=True))

        # ended_effect = pddl.durative.TimedEffect(self.ended_pred, [], "start", a2, negated=False)
        # acquire_lock = pddl.durative.TimedEffect(self.lock_pred, [], "start", a2, negated=False)
        # release_lock = pddl.durative.TimedEffect(self.lock_pred, [], "end", a2, negated=True)
        
        # if action.replan:
        #     a2.replan = action.replan.copy(new_scope=a2)

        # add_conds = []
        # add_effects = []

        # if action.name.startswith("ignore-preference") and "fullfill" not in action.name:
        #     add_conds.append(lock_cond)
        #     add_effects += [ended_effect, acquire_lock, release_lock]
        # else:
        #     add_conds.append(ended_cond)

        # cond = None
        # if add_conds:
        #     cond = pddl.Conjunction(add_conds, a2)
        # if action.precondition:
        #     orig = pddl.durative.TimedCondition("start", action.precondition.copy(new_scope=a2))
        #     if cond:
        #         cond.parts.append(orig)
        #     else:
        #         cond = orig

        # effect = None
        # if add_effects:
        #     effect = pddl.ConjunctiveEffect(add_effects, a2)
        # if action.effect:
        #     orig = action.effect.visit(effect_visitor)
        #     if effect:
        #         effect.parts.append(orig)
        #     else:
        #         effect = orig

        # a2.precondition = cond
        # a2.precondition.set_scope(a2)
        # a2.effect = effect
        # a2.effect.set_scope(a2)
            
        # return a2

    def translate_domain(self, _domain):
        dom = _domain.copy_skeleton()
        if "durative-actions" not in dom.requirements:
            dom.add_requirement("durative-actions")
        dom.requirements.discard("action-costs")
        
        dom.predicates.add(self.lock_pred)
        dom.predicates.add(self.ended_pred)

        dom.clear_actions()
        for a in _domain.get_action_like():
            dom.add_action(self.translate_action(a, dom))
            
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        return dom
    
    def translate_problem(self, _problem):
        p2 = pddl.translators.Translator.translate_problem(self, _problem)
        p2.optimization = None
        p2.opt_func = None
        return p2
