import itertools, time

import config, constants
import assertions, macros
import pddl
from  pddl import state, mapl
import statistics

from utils import Enum

log = config.logger("planner")

PlanningStatusEnum = Enum("TASK_CHANGED", "RUNNING", "PLAN_AVAILABLE", "PLANNING_FAILURE", "INTERRUPTED")    

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
            
        if writer:
            self.writer = writer
        else:
            self.writer = pddl.writer.Writer()

    def translate(self, problem, domain=None):
        if not self.compiler:
            if not domain:
                domain = problem.domain
            return domain, problem
        
        t0 = time.time()
        if domain is not None and domain != problem.domain:
            tr_dom = self.compiler.translate(domain)
            tr_prob = self.compiler.translate(problem)
        else:
            tr_prob = self.compiler.translate(problem)
            tr_dom = tr_prob.domain
        log.debug("total time for translation: %f", time.time()-t0)
        return tr_dom, tr_prob

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

class ADLOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = pddl.translators.ADLCompiler()
    
class TFDOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = TemporalTranslator()

class FDOutput(PDDLOutput):
    def __init__(self):
        self.writer = pddl.writer.Writer()
        self.compiler = pddl.translators.ADLCompiler()
        
    def write(self, problem, domain=None, domain_fn=None, problem_fn=None, mutex_fn=None):
        tr_dom, tr_prob = self.translate(problem, domain)

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

        for function in oldprob.functions:
            pred = prob.predicates.get(function.name, function.args+[function.type])
            fluents_combinations = state.product(*(prob.get_all_objects(a.type) for a in function.args))
            for c in fluents_combinations:
                group = []
                for v in prob.get_all_objects(function.type):
                    #print pddl.Literal(pred, c+[v]).pddl_str()
                    group.append(pddl.Literal(pred, itertools.chain(c, [v])))
                groups.append(group)
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
    def __init__(self):
        self.depends = [pddl.translators.ModalPredicateCompiler(remove_replan=True), pddl.translators.PreferenceCompiler()]
        self.lock_pred = pddl.Predicate("locked", [])

    def translate_action(self, action, domain=None):
        tct = pddl.Term(pddl.builtin.total_cost,[])

        affected_vars = set()
        cond_effects = []
        
        @pddl.visitors.copy
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.SimpleEffect):
                if eff.predicate == pddl.builtin.increase and eff.args[0] == tct:
                    return False
                if eff.predicate == pddl.assign:
                    affected_vars.add((eff.args[0].function, ) + tuple(eff.args[0].args))
                    return pddl.SimpleEffect(pddl.builtin.change, eff.args[:])
                if eff.predicate == pddl.builtin.num_assign:
                    affected_vars.add((eff.args[0].function, ) + tuple(eff.args[0].args))
                    return pddl.SimpleEffect(pddl.builtin.num_change, eff.args[:])
                affected_vars.add((eff.predicate, ) + tuple(eff.args))
                return pddl.durative.TimedEffect(eff.predicate, eff.args[:], "end", negated=eff.negated)
            if isinstance(eff, pddl.ConditionalEffect):
                eff2 = pddl.ConditionalEffect(None, results[0])
                eff2.condition = pddl.durative.TimedCondition("all", eff.condition.copy())
                cond_effects.append(eff2)
                return eff2

        if isinstance(action, pddl.durative.DurativeAction):
            return action.copy(newdomain=domain)

        total_costs = action.get_total_cost()
        if total_costs:
            dc = pddl.durative.DurationConstraint(total_costs)
        else:
            dc = pddl.durative.DurationConstraint(pddl.Term(1))
        args = [pddl.Parameter(p.name, p.type) for p in action.args]
        a2 = pddl.durative.DurativeAction(action.name, args, [dc], None, None, domain)

        lock_cond = pddl.durative.TimedCondition("start", pddl.LiteralCondition(self.lock_pred, [], a2, negated=True))
        
        if action.replan:
            a2.replan = action.replan.copy(new_scope=a2)
        if action.precondition:
            a2.precondition = pddl.Conjunction([lock_cond, pddl.durative.TimedCondition("start", action.precondition.copy(new_scope=a2))])
        else:
            a2.precondition = lock_cond

        acquire_lock = pddl.durative.TimedEffect(self.lock_pred, [], "start", a2, negated=False)
        release_lock = pddl.durative.TimedEffect(self.lock_pred, [], "end", a2, negated=True)
        effect = pddl.ConjunctiveEffect([acquire_lock, release_lock], a2)
            
        if action.effect:
            effect.parts.append(action.effect.visit(effect_visitor))
            
        a2.effect = effect
        a2.effect.set_scope(a2)
            
        return a2

    def translate_domain(self, _domain):
        dom = pddl.Domain(_domain.name, _domain.types.copy(), set(_domain.constants), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.requirements.add("durative-actions")
        dom.requirements.discard("action-costs")
        
        dom.predicates.add(self.lock_pred)

        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.observe = [self.translate_action(o, dom) for o in _domain.observe]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        return pddl.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, None, None)
