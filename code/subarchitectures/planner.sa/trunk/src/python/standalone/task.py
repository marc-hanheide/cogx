import itertools

import config, constants#, assertions
import state_new as state
import mapl_new as mapl
import mapl_new.mapltypes as types
import mapl_new.predicates as predicates
import mapl_new.effects as effects
import statistics
#import mapl_new.writer
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
            self._mapldomain = mapltask
            self.create_initial_state()
            #self.add_assertions()


    def __get_mapltask(self):
        if self.is_dirty():
            new_init = [ f.asLiteral(useEqual=True) for f in self.get_state().iterfacts() ]
            self._mapltask = mapl.problem.Problem(self._mapltask.name, self._mapltask.objects, new_init, self._mapltask.goal, self._mapldomain)
        return self._mapltask

    def __set_mapltask(self, mapltask):
        self._mapltask = mapltask

    mapltask = property(__get_mapltask, __set_mapltask)

    # def add_assertions(self):
    #     new_assertions = []
    #     for a in itertools.chain(self._mapldomain.actions, self._mapldomain.sensors):
    #         ast = assertions.to_assertion(a, self._mapldomain)
    #         if ast:
    #             new_assertions.append(ast)

    #     self._mapldomain.actions += new_assertions

    def create_initial_state(self):
        s = state.State([], self._mapltask)
        for i in self._mapltask.init:
            #determinise probabilistic init conditions
            if isinstance(i, effects.ProbabilisticEffect):
                for p, eff in i.effects:
                    facts = s.getEffectFacts(eff)
                    for svar, value in facts:
                        if not isinstance(svar, mapl.predicates.Predicate) and svar.modality is None:
                            id_var = svar.asModality(mapl.predicates.indomain, [value])
                            s[id_var] = mapl.types.TRUE
            else:
                s.set(state.Fact.fromLiteral(i))
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

    def pddl_domain_str(self):
        w = PDDLWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_domain(self._mapldomain))

    def pddl_problem_str(self):
        w = PDDLWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_problem(self.mapltask))

    def tfd_domain_str(self):
        w = TFDWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_domain(self._mapldomain))

    def tfd_problem_str(self):
        w = TFDWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_problem(self.mapltask))
    
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


                
class PDDLWriter(mapl.writer.MAPLWriter):
    def __init__(self, modalPredicates):
        self.modal = modalPredicates     
    
    def write_type(self, type):
        if isinstance(type, types.ProxyType):
            return mapl.writer.MAPLWriter.write_type(self, type.effectiveType())
        return mapl.writer.MAPLWriter.write_type(self, type)

    def write_term(self, term):
        if isinstance(term, (mapl.predicates.ConstantTerm, mapl.predicates.VariableTerm)):
            return term.object.name
        assert False, "No function terms in ff-pddl: %s" % str(term)
    
    def write_literal(self, literal):
        args=[]
        name_elems = []
        if literal.predicate not in mapl.predicates.assignmentOps+[mapl.predicates.equals]:
            name_elems.append(literal.predicate.name)
            
        for arg in literal.args:
            if arg == mapl.types.UNKNOWN:
                return ""
            if isinstance(arg, mapl.predicates.FunctionTerm):
                name_elems.append(arg.function.name)
                assert not any(map(lambda a: isinstance(a, mapl.predicates.FunctionTerm), arg.args)), "no nested function allowed!"
                args += arg.args
            else:
                args.append(arg)

        #We probably have a equal between constants
        if not name_elems:
            name_elems.append("=")
                
        argstr = " ".join(map(lambda t: self.write_term(t), args))
        result = "(%s %s)" % ("-".join(name_elems), argstr)
        if literal.negated:
            result = "(not %s)" % result
        return result

    def write_predicate(self, pred):
        return "(%s %s)" % (pred.name, self.write_typelist(pred.args))
    
    def write_predicates(self, preds, functions):
        strings = []
        domain_modal = []
        for pred in preds:
            if pred.builtin:
                continue
            
            if any(map(lambda arg: isinstance(arg.type, types.FunctionType), pred.args)):
                domain_modal.append(pred)
            else:
                strings.append(self.write_predicate(pred))
                
        for func in functions:
            if func.builtin:
                continue
            strings.append(";; Function %s" % func.name)
            pred = predicates.Predicate(func.name, func.args+[predicates.Parameter("?val", func.type)])
            strings.append(self.write_predicate(pred))
            for mod in self.modal + domain_modal:
                args =[]
                for arg in mod.args:
                    if isinstance(arg.type, types.FunctionType):
                        args += func.args
                    elif isinstance(arg.type, types.ProxyType):
                        args.append(types.Parameter(arg.name, func.type))
                    else:
                        args.append(arg)

                pred =  predicates.Predicate("%s-%s" % (mod.name, func.name), args)
                strings.append(self.write_predicate(pred))

        return self.section(":predicates", strings)

    def write_functions(self, funcs):
        return []

    def write_modal_action(self, action, newname, newargs):
        def factVisitor(cond, results):
            if isinstance(cond, mapl.conditions.LiteralCondition):
                if cond.predicate == predicates.equals:
                    return [(cond.args[0], cond.args[1])]
                return []
            else:
                return sum(results, [])
                
        strings = [newname]
        strings += self.section(":parameters", ["(%s)" % self.write_typelist(newargs)], parens=False)

        read_facts = []
        
        if action.precondition:
            read_facts += action.precondition.visit(factVisitor)
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)
        if action.replan:
            read_facts += action.replan.visit(factVisitor)
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)

        previous_values = {}
        for term, value in read_facts:
            if term in previous_values:
                previous_values[term] = None
            else:
                previous_values[term] = value
        
        def effectsVisitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate == predicates.assign:
                    term = eff.args[0]
                    if term in previous_values and previous_values[term] is not None:
                        oldval = previous_values[term]
                        return [effects.SimpleEffect(predicates.assign, [term, oldval], negated=True), eff]
                    else:
                        param = types.Parameter("?oldval", term.function.type)
                        negeffect = effects.SimpleEffect(predicates.assign, [term, predicates.Term(param)], negated=True)
                        return [effects.UniversalEffect([param], [negeffect], None), eff]
                return [eff]
            else:
                eff.effects = sum(results, [])

        new_effects = []
        for e in action.effects:
            new_effects += e.copy().visit(effectsVisitor)

        strings += self.section(":effect", self.write_effects(new_effects), parens=False)

        return self.section(":action", strings)
    
    def write_action(self, action, functions):
        args = action.agents + action.args + action.vars
        func_arg, compiled_args = compile_modal_args(args, functions)
        
        if func_arg is None:
            return self.write_modal_action(action, action.name, args)
        
        strings=[]

        for f, cargs in compiled_args:
            action.instantiate({func_arg : predicates.FunctionTerm(f, [predicates.Term(a) for a in f.args])})
            strings += self.write_modal_action(action, "%s-%s" % (action.name, f.name), cargs)
            action.uninstantiate()
            strings.append("")

        return strings
        

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
        strings = [action.name]
        args = action.agents + action.args + action.vars
        strings += self.section(":parameters", ["(%s)" % self.write_typelist(args)], parens=False)

        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)
            
        if isinstance(action.sense, predicates.Literal):
            term = action.sense.args[0]
        else:
            term = action.sense

        strings += self.section(":effect", self.write_effect(action.knowledge_effect()), parens=False)
        return self.section(":action", strings)

    def write_init(self, inits):
        strings = []
        for i in inits:
            if not i.negated:
                strings.append(self.write_literal(i))

        return self.section(":init", strings)
    
    def write_domain(self, domain):
        strings = ["(define (domain %s)" % domain.name]
        strings.append("")
        #strings.append("(:requirements %s)" % " ".join(map(lambda r: ":"+r, domain.requirements)))
        strings.append("(:requirements :adl)")
        strings.append("")
        strings += self.write_types(domain.types.itervalues())
        
        if domain.constants:
            strings.append("")
            strings += self.write_objects("constants", domain.constants)

        strings.append("")
        strings += self.write_predicates(domain.predicates, domain.functions)
        
        for a in domain.axioms:
            strings.append("")
            strings += self.write_axiom(a, domain.functions)
        for s in domain.sensors:
            strings.append("")
            strings += self.write_sensor(s)
        for a in domain.actions:
            strings.append("")
            strings += self.write_action(a, domain.functions)

        strings.append("")
        strings.append(")")
        return strings


class TFDWriter(mapl.writer.MAPLWriter):
    def __init__(self, modalPredicates):
        self.modal = modalPredicates

    def write_type(self, type):
        if isinstance(type, types.ProxyType):
            return mapl.writer.MAPLWriter.write_type(self, type.effectiveType())
        return mapl.writer.MAPLWriter.write_type(self, type)
    
    def write_term(self, term):
        if isinstance(term, (predicates.FunctionVariableTerm)) and term.isInstantiated():
            return self.write_term(term.getInstance())
        else:
            return mapl.writer.MAPLWriter.write_term(self, term)
        
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
                    if isinstance(arg.type, types.FunctionType) and func.type.equalOrSubtypeOf(arg.type.type):
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

    def write_durative_action_real(self, name, params, duration, condition, effects):
        strings = [name]
        strings += self.section(":parameters", ["(%s)" % self.write_typelist(params)], parens=False)
        strings += self.section(":duration", self.write_durations(duration), parens=False)
        
        if condition:
            strings += self.section(":condition", self.write_condition(condition), parens=False)

        strings += self.section(":effect", self.write_effects(effects), parens=False)
        return self.section(":durative-action", strings)
    
    def write_durative_action(self, action, functions):
        args = action.agents + action.args + action.vars
        func_arg, compiled_args = compile_modal_args(args, functions)

        condition = action.precondition
        if action.replan:
            replan = conditions.TimedCondition("start", action.replan)
            if condition:
                condition = mapl.conditions.Conjunction([replan, prec])
            else:
                condition = replan
                    
        if func_arg is None:
            return self.write_durative_action_real(action.name, args, action.duration, condition, action.effects)
        strings=[]

        for f, cargs in compiled_args:
            action.instantiate({func_arg : predicates.FunctionTerm(f, [predicates.Term(a) for a in f.args])})
            strings += self.write_durative_action_real("%s-%s" % (action.name, f.name), cargs, action.duration, condition, action.effects)
            action.uninstantiate()
            strings.append("")

        return strings

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
         
        return self.write_durative_action_real(action.name, args, [duration], condition, teff)
        
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
            strings += self.write_durative_action(a, domain.functions)

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
                if func.builtin or not func.type.equalOrSubtypeOf(a.type.type):
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
