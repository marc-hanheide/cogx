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
        w = writer_class(mapl.mapl.modal_predicates)
        return "\n".join(w.write_problem(self.mapltask))

    def domain_str(self, writer_class):
        w = writer_class(mapl.mapl.modal_predicates)
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
    def __init__(self, modalPredicates):
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


class TFDWriter(mapl.writer.Writer):
    def __init__(self, modalPredicates):
        self.modal = modalPredicates

    def write_type(self, type):
        if isinstance(type, types.ProxyType):
            return mapl.writer.Writer.write_type(self, type.effective_type())
        return mapl.writer.Writer.write_type(self, type)
    
    def write_term(self, term):
        if isinstance(term, (predicates.FunctionVariableTerm)) and term.is_instantiated():
            return self.write_term(term.get_instance())
        else:
            return mapl.writer.Writer.write_term(self, term)
        
    def write_literal(self, literal):
        args = []
        name_elems = [literal.predicate.name]
        if literal.predicate in self.modal:
            for arg in literal.args:
                if isinstance(arg, mapl.predicates.FunctionTerm):
                    name_elems.append(arg.function.name)
                    args += arg.args
                else:
                    args.append(arg)
        else:
            args = literal.args

        argstr = " ".join(map(lambda t: self.write_term(t), args))
        return "(%s %s)" % ("-".join(name_elems), argstr)

    def write_predicate(self, pred):
        return "(%s %s)" % (pred.name, self.write_typelist(pred.args))
    
    def write_predicates(self, preds, functions):
        strings = []
        for pred in preds:
            if not pred.builtin:
                strings.append(self.write_predicate(pred))

        strings.append(";; MAPL-defined predicates")
        for pred in mapl.predicates.mapl_nonmodal_predicates:
                strings.append(self.write_predicate(pred))
                
        for func in functions:
            if func.builtin:
                continue
            strings.append(";; Function %s" % func.name)
            for mod in self.modal:
                args =[]
                function_match = False
                for arg in mod.args:
                    if isinstance(arg.type, types.FunctionType) and func.type.equal_or_subtype_of(arg.type.type):
                        function_match = True
                        args += func.args
                    elif isinstance(arg.type, types.ProxyType):
                        args.append(types.Parameter(arg.name, func.type))
                    else:
                        args.append(arg)
                if function_match:
                    pred =  predicates.Predicate("%s-%s" % (mod.name, func.name), args)
                    strings.append(self.write_predicate(pred))

        return self.section(":predicates", strings)

    def write_durative_action_real(self, name, params, duration, condition, replan, effects):
        strings = [name]
        strings += self.section(":parameters", ["(%s)" % self.write_typelist(params)], parens=False)
        strings += self.section(":duration", self.write_durations(duration), parens=False)
        
        if condition:
            strings += self.section(":condition", self.write_condition(condition), parens=False)
        if replan:
            strings += self.section(":replan", self.write_condition(replan), parens=False)

        strings += self.section(":effect", self.write_effects(effects), parens=False)
        return self.section(":durative-action", strings)
    
    def write_durative_action(self, action, functions):
        args = action.agents + action.args + action.vars
        func_arg, compiled_args = compile_modal_args(args, functions)

        replan = None
        if action.replan:
            replan = mapl.conditions.TimedCondition("start", action.replan)
                    
        if func_arg is None:
            return self.write_durative_action_real(action.name, args, action.duration, action.precondition, replan, action.effects)
        strings=[]

        for f, cargs in compiled_args:
            action.instantiate({func_arg : predicates.FunctionTerm(f, [predicates.Term(a) for a in f.args])})
            strings += self.write_durative_action_real("%s-%s" % (action.name, f.name), cargs, action.duration, condition, replan, action.effects)
            action.uninstantiate()
            strings.append("")

        return strings
    
    def write_action(self, action, functions):
        def effect_visitor(eff, results):
            if isinstance(eff, mapl.effects.SimpleEffect):
                if eff.predicate == mapl.predicates.assign:
                    return mapl.effects.SimpleEffect(mapl.predicates.change, eff.args)
                else:
                    return mapl.effects.TimedEffect(eff.predicate, eff.args, "start", eff.negate)
            elif isinstance(eff, list):
                return results
            else:
                return eff.copy(new_parts = results)
        
        condition = None
        replan = None
        if action.precondition:
            condition = mapl.conditions.TimedCondition("start", action.precondition)
        if action.replan:
            replan = mapl.conditions.TimedCondition("start", action.replan)
            
        effects = []
        for eff in action.effects:
            effects.append(eff.visit(effect_visitor))

        new = mapl.actions.DurativeAction(action.name, action.agents, action.args, action.vars, None, condition, replan, effects, action.parent)
        duration_term = mapl.predicates.FunctionTerm(mapl.predicates.eq, ["?duration", 1.0], new)
        new.duration = [mapl.actions.DurationConstraint(duration_term, None)]
        return self.write_durative_action(new.copy(), functions)
    
    def write_axiom_real(self, name, params, condition):
        strings = ["(%s %s)" % (name, self.write_typelist(params))]
        strings += self.write_condition(condition)
        return self.section(":derived", strings)

    def write_axiom(self, axiom, functions):
        func_arg, compiled_args = compile_modal_args(axiom.args, functions)

        if func_arg is None:
            return self.write_axiom_real(axiom.predicate.name, axiom.args, axiom.condition)
        strings=[]

        for f, cargs in compiled_args:
            axiom.instantiate({func_arg : predicates.FunctionTerm(f, [predicates.Term(a) for a in f.args])})
            strings += self.write_axiom_real("%s-%s" % (axiom.predicate.name, f.name), cargs, axiom.condition)
            axiom.uninstantiate()
            strings.append("")

        return strings
    
    def write_sensor(self, action):
        args = action.agents + action.args + action.vars
        duration = mapl.actions.DurationConstraint(predicates.Term(1.0), None)
        if action.precondition:
            condition = mapl.conditions.TimedCondition("all", action.precondition)
        else:
            condition = None
        keff = action.knowledge_effect()
        teff = [effects.TimedEffect(keff.predicate, keff.args, "end")]
         
        return self.write_durative_action_real(action.name, args, [duration], condition, None, teff)
        
    def write_domain(self, domain):
        strings = ["(define (domain %s)" % domain.name]
        strings.append("")
        strings.append("(:requirements %s)" % " ".join(map(lambda r: ":"+r, filter(lambda r: r != "mapl", domain.requirements))))
        strings.append("")
        strings += self.write_types(domain.types.itervalues())
        
        if domain.constants:
            strings.append("")
            strings += self.write_objects("constants", domain.constants)

        strings.append("")
        strings += self.write_predicates(domain.predicates, domain.functions)
        strings += self.write_functions(domain.functions)
        
        for a in domain.axioms:
            strings.append("")
            strings += self.write_axiom(a, domain.functions)
        for s in domain.sensors:
            strings.append("")
            strings += self.write_sensor(s)
        for a in domain.actions:
            strings.append("")
            if isinstance(a, mapl.actions.DurativeAction):
                strings += self.write_durative_action(a, domain.functions)
            else:
                strings += self.write_action(a, domain.functions)

        strings.append("")
        strings.append(")")
        return strings

def compile_modal_args(args, functions):
    func_arg = None
    funcs = []
    pref = []
    suff = []

    for a in args:
        if isinstance(a.type, types.FunctionType):
            func_arg = a
            for func in functions:
                if func.builtin or not func.type.equal_or_subtype_of(a.type.type):
                    continue
                funcs.append(func)
        else:
            if func_arg:
                suff.append(a)
            else:
                pref.append(a)
            
    compiled = []
    for f in funcs:
        compiled.append((f, pref + f.args + suff))
        
    return func_arg, compiled
