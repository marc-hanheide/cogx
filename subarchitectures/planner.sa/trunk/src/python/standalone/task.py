import itertools, time

import config, constants
import assertions, macros
import mapl_new as mapl
import mapl_new.state as state
import mapl_new.mapltypes as types
import mapl_new.predicates as predicates
import mapl_new.effects as effects
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

    def __init__(self, taskID=0, mapltask = None):
        """Initialise public and private fields."""
        # public
        self.taskID = taskID
        self.statistics = statistics.Statistics(statistics_defaults)
        self._mapltask = mapltask
        self._mapldomain = None
        self._action_blacklist = None
        self._action_whitelist = None
        self._plan = None
        # private
        self._state = None
        self.planning_status = PlanningStatusEnum.TASK_CHANGED
        
        if mapltask:
            self._mapldomain = mapltask.domain
            self.create_initial_state()
            self.add_assertions()


    def __get_mapltask(self):
        if self.is_dirty():
            new_init = [ f.as_literal(useEqual=True) for f in self.get_state().iterfacts() ]
            self._mapltask = mapl.mapl.MAPLProblem(self._mapltask.name, self._mapltask.objects, new_init, self._mapltask.goal, self._mapldomain, self._mapltask.optimization, self._mapltask.opt_func)
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
        for a in itertools.chain(self._mapldomain.actions, self._mapldomain.sensors):
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
            if isinstance(i, effects.ProbabilisticEffect):
                for p, eff in i.effects:
                    facts = s.get_effect_facts(eff)
                    for svar, value in facts:
                        if not isinstance(svar, mapl.predicates.Predicate) and svar.modality is None:
                            id_var = svar.as_modality(mapl.mapl.i_indomain, [value])
                            s[id_var] = mapl.TRUE
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

    def problem_str(self, writer_class):
        w = writer_class()
        return "\n".join(w.write_problem(self.mapltask))

    def domain_str(self, writer_class):
        w = writer_class()
        return "\n".join(w.write_domain(self._mapldomain))
    
    def load_mapl_domain(self, domain_file):
        log.info("Loading MAPL domain %s.", domain_file)
        self._mapldomain = mapl.load_domain(domain_file)
        
    def load_mapl_problem(self, task_file, agent_name=None):
        log.info("Loading MAPL problem %s.", task_file)
        self.mapltask = mapl.load_problem(task_file, self._mapldomain)
        self.create_initial_state()

        self._agent_name = agent_name

    def parse_mapl_problem(self, problem_str, agent_name=None):
        self._mapltask = mapl.parse_problem(problem_str, self._mapldomain)
        self.create_initial_state()

        self._agent_name = agent_name

    def load_mapl_task(self, task_file, domain_file, agent_name=None):
        self.load_mapl_domain(domain_file)
        self.load_mapl_problem(task_file, agent_name)


class PDDLWriter(mapl.writer.Writer):
    def __init__(self):
        self.compiler = mapl.translators.ADLCompiler()
        #self.compiler = mapl.translators.ModalPredicateCompiler()
        
    def write_problem(self, problem):
        t0 = time.time()
        p2 = self.compiler.translate(problem)
        log.debug("total time for translation: %f", time.time()-t0)
        return mapl.writer.Writer.write_problem(self, p2)

    def write_domain(self, domain):
        dom = self.compiler.translate(domain)
        return mapl.writer.Writer.write_domain(self, dom)

class TFDWriter(PDDLWriter):
    def __init__(self):
        self.compiler = TemporalTranslator()

    
class FDWriter(PDDLWriter):
    def write_problem(self, problem):
        t0 = time.time()
        p2 = self.compiler.translate(problem)
        self.mutex_groups = self.get_mutex_groups(p2, problem)
        log.debug("total time for translation: %f", time.time()-t0)
        return mapl.writer.Writer.write_problem(self, p2)

    def get_mutex_groups(self, prob, oldprob):
        groups = []
        for function in oldprob.functions:
            pred = prob.predicates.get(function.name, function.args+[function.type])
            fluents_combinations = mapl.state.product(*(prob.get_all_objects(a.type) for a in function.args))
            for c in fluents_combinations:
                group = []
                for v in prob.get_all_objects(function.type):
                    #print mapl.Literal(pred, c+[v]).pddl_str()
                    group.append(mapl.Literal(pred, itertools.chain(c, [v])))
                groups.append(group)
        return groups
    
    def write_mutex(self, mutex_groups):
        groups = []
        for g in mutex_groups:
            strings = []
            for atom in g:
                strings.append(self.write_literal(atom))
            groups += self.section("", strings)
        return self.section("mutex", groups)
        

class TemporalTranslator(mapl.translators.Translator):
    def __init__(self):
        self.depends = [mapl.translators.ModalPredicateCompiler(remove_replan=True)]

    def translate_action(self, action, domain=None):

        @mapl.visitors.copy
        def effect_visitor(eff, results):
            if isinstance(eff, mapl.SimpleEffect):
                if eff.predicate == mapl.assign:
                    return mapl.SimpleEffect(mapl.builtin.change, eff.args[:])
                if eff.predicate == mapl.builtin.num_assign:
                    return mapl.SimpleEffect(mapl.builtin.num_change, eff.args[:])
                return mapl.durative.TimedEffect(eff.predicate, eff.args[:], "end", negated=eff.negated)
            if isinstance(eff, mapl.ConditionalEffect):
                eff2 = mapl.ConditionalEffect(None, results[0])
                eff2.condition = mapl.durative.TimedCondition("start", eff.condition.copy())
                return eff2
        
        if isinstance(action, mapl.durative.DurativeAction):
            return action.copy(newdomain=domain)
        
        dc = mapl.durative.DurationConstraint(mapl.Term(1))
        args = [types.Parameter(p.name, p.type) for p in action.args]
        a2 = mapl.durative.DurativeAction(action.name, args, [dc], None, None, domain)
        
        if action.replan:
            a2.replan = action.replan.copy(new_scope=a2)
        if action.precondition:
            a2.precondition = mapl.durative.TimedCondition("start", action.precondition.copy(new_scope=a2))
            
        a2.effect = action.effect.visit(effect_visitor)
        a2.effect.set_scope(a2)
            
        return a2
