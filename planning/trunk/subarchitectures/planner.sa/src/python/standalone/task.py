import constants
import state_new as state
import mapl_new as mapl
import mapl_new.mapltypes as types
import mapl_new.predicates as predicates
import mapl_new.effects as effects
#import mapl_new.writer
from utils import Enum

PlanningStatusEnum = Enum("TASK_CHANGED", "RUNNING", "PLAN_AVAILABLE", "PLANNING_FAILURE", "INTERRUPTED")    

class Task(object):
    """
    """

    def __init__(self, taskID=0):
        """Initialise public and private fields."""
        # public
        self.taskID = taskID
        self._mapltask = None
        self._mapldomain = None
        self._action_blacklist = None
        self._action_whitelist = None
        self._plan = None
        # private
        self._new_plan_callback = None
        self._state = None
        self._planning_status = PlanningStatusEnum.TASK_CHANGED
        self._change_detection_activated = False

    def set_planning_status(self, status, trigger_planning=True):
        """Mark task as changed and call planner if necessary."""
        assert status in PlanningStatusEnum.values()
        self._planning_status = status
        if status == PlanningStatusEnum.TASK_CHANGED:
            print "Status of task %s was changed to %s. This may trigger planner activity." % (self.taskID, status)
        else:
            print "Status of task %s was changed to %s." % (self.taskID, status)
        if trigger_planning:
            self.update_planning()
        #if status == PlanningStatusEnum.PLAN_AVAILABLE:
        #    print "Heureka! A plan was found:\n%s" % self._plan

    def get_planning_status(self):
        return self._planning_status

    def mark_changed(self):
        self.set_planning_status(PlanningStatusEnum.TASK_CHANGED)

    def is_dirty(self):
        """Check if task has been modified so that the plan is possibly outdated."""
        return self.get_planning_status() == PlanningStatusEnum.TASK_CHANGED

    def activate_change_dectection(self):
        """Activated automatic change detection and calling of the planner.
        Activation may lead to immediate updating of the plan if the task was modified
        while change detection was suspended."""
        self._change_detection_activated = True
        print "Change detection was activated for task %s. This may trigger planner activity." % self.taskID
        self.update_planning()
    
    def suspend_change_dectection(self):
        """Do not update the plan immediately upon changes.  This is useful if 
        several changes are known to be made in a row.  Then planning will only be 
        triggered upon calling activate_change_dectection().""" 
        self._change_detection_activated = False

    def change_detection_activated(self):
        """Is change detection currently activated?"""
        return self._change_detection_activated

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
            self.set_planning_status(PlanningStatusEnum.TASK_CHANGED)

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
                self.set_planning_status(PlanningStatusEnum.PLANNING_FAILURE)
            else:
                self.set_planning_status(PlanningStatusEnum.PLAN_AVAILABLE)
            if self._new_plan_callback:
                self._new_plan_callback(self)

    def update_planning(self):
        """If the task is registered with a planner, the plan will be updated,
        but only if the task has been modified and  if change detection is activated."""
        if self.change_detection_activated() and self.is_dirty() and self.planner:
            self.planner.continual_planning(self)

    def set_plan_callback(self, fn):
        self._new_plan_callback = fn

    def pddl_domain_str(self):
        w = PDDLWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_domain(self._mapldomain))
        #return PDDLTask.from_MAPL_task(self).pddl_domain_str()

    def pddl_problem_str(self):
        w = PDDLWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_problem(self._mapltask))
        
        #return PDDLTask.from_MAPL_task(self).pddl_problem_str()

    def tfd_domain_str(self):
        w = TFDWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_domain(self._mapldomain))
        #return PDDLTask.from_MAPL_task(self).pddl_domain_str()

    def tfd_problem_str(self):
        w = TFDWriter(predicates.mapl_modal_predicates)
        return "\n".join(w.write_problem(self._mapltask))
        
        #return PDDLTask.from_MAPL_task(self).pddl_problem_str()
    
    def load_mapl_domain(self, domain_file):
        print "Loading MAPL domain %s." % domain_file
        self._mapldomain = mapl.load_domain(domain_file)
        
    def load_mapl_problem(self, task_file, agent_name=None):
        print "Loading MAPL problem %s." % task_file
        self._mapltask = mapl.load_problem(task_file, self._mapldomain)
        self._state = state.State.fromProblem(self._mapltask)

        self._agent_name = agent_name

    def parse_mapl_problem(self, problem_str, agent_name=None):
        self._mapltask = mapl.parse_problem(problem_str, self._mapldomain)
        self._state = state.State.fromProblem(self._mapltask)

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
        for pred in preds:
            if not pred.builtin:
                strings.append(self.write_predicate(pred))
                
        for func in functions:
            if func.builtin:
                continue
            strings.append(";; Function %s" % func.name)
            pred = predicates.Predicate(func.name, func.args+[predicates.Parameter("?val", func.type)])
            strings.append(self.write_predicate(pred))
            for mod in self.modal:
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
        
        if action.replan:
            read_facts += action.replan.visit(factVisitor)
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.precondition:
            read_facts += action.precondition.visit(factVisitor)
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)

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


class PDDLTask(Task):
    @classmethod
    def from_MAPL_task(cls, mapl_task):
        """ compile a MAPL task into a corresponding PDDL representation. """
        import copy
        t = copy.deepcopy(mapl_task)  # only a shallow copy. is that enough?
        t.__class__ = PDDLTask
        t.add_implicit_MAPL_types(constants.MAPL_BASE_ONTOLOGY.split())
        #t._type_tree.closure()
        for pred in constants.MAPL_BASE_PREDICATES:
            t.add_implicit_MAPL_predicate(pred)
        new_objects = types.parse_typed_list(constants.MAPL_BASE_OBJECTS.split())
        t.add_implicit_MAPL_constants(new_objects)
        t._operators = [a.translate2pddl(t._state_variables) for a in t._operators]
        return t
    
    def add_implicit_MAPL_predicate(self, pred_decl_str):
        alist = pred_decl_str.split()
        pred_name = alist[0]
        if pred_name not in self._state_variables:
            new_pred = predicates.Predicate.parse(alist)
            self._state_variables[pred_name] = new_pred

    def pddl_domain_str(self):
        elmts = self._pddl_domain_gen()
        #print list(elmts)
        return "\n".join(elmts)

    def pddl_problem_str(self):
        elmts = self._pddl_problem_gen()
        #print list(elmts)
        return "\n".join(elmts)

    def _pddl_domain_gen(self, keep_assertions=True):
        yield ";; PDDL domain '%s' (compiled from MAPL)" % self._task_name
        yield ""
        yield "(define (domain %s)" % self._domain_name
        yield self._requirements.pddl_str()
        # types
        yield "(:types"
        yield self._type_tree.to_str()
        yield ")"
        # predicates
        yield "(:predicates"
        agts = [types.TypedObject("?agt0", "agent")]
        for pred in self._state_variables.values():
            yield "  ;; predicate: %s" % pred.name
            declarations = []
#             if pred.name == "=":
#                 continue
            declarations.append(pred.pddl_str())
            kpred = pred.knowledge_pred(agts)
            declarations.append(kpred.pddl_str())
            declarations.append(kpred.pddl_str(k_axiom=True))
            for decl in declarations:
                yield "  (%s)" % decl
        yield ")"
        #constants
        yield "(:constants"
        for obj in self._constants:
            yield "  %s - %s" % (obj.name, obj.type)
        yield ")"
        # axioms
        for axiom in self._axioms:
            for line in axiom.pddl_gen():
                yield line
        # actions
        for action in self._operators:
            for line in action.pddl_gen(keep_assertions):
                yield line
        # sensors
        for sensor in self._sensors:
            for line in sensor.pddl_gen(keep_assertions):
                yield line
        yield ")"

    def _pddl_problem_gen(self):
        yield ";; PDDL problem '%s' (compiled from MAPL)" % self._domain_name
        yield "(define (problem %s)" % self._task_name
        yield "(:domain %s)" % self._domain_name
        #objects
        yield "(:objects"
        for obj in self._objects:
            yield "  %s - %s" % (obj.name, obj.type)
        yield ")"
        #init

        yield "(:init"
        for fact in self.get_state().get_facts():
            if fact.is_equality_constraint():
                continue
            factstr = fact.as_pddl_str(use_direct_k=True)
            yield "  %s" % factstr
        yield ")"
        yield "(:goal"
        yield self._goal.pddl_str()
        yield "))"
