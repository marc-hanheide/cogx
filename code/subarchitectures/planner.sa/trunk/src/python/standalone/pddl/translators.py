from collections import defaultdict
import time

from scope import Scope
import predicates, conditions, effects, actions, axioms, domain, problem, utils
import visitors
import mapltypes as types
import builtin
from builtin import *
from predicates import *

class TranslatorAnnotations(dict):
    pass

def requires(*requirements):
    import functools
    def decorator(f):
        @functools.wraps(f)
        def wrapper(self, *args, **kwargs):
            for a in args:
                if isinstance(a, domain.Domain):
                    #print requirement, map(str, a.requirements)
                    if not any(r in a.requirements for r in requirements):
                        #print "pass"
                        return self.pass_trough_domain(a, **kwargs)
                if isinstance(a, problem.Problem):
                    #print requirement, map(str, a.domain.requirements)
                    if not any(r in a.domain.requirements for r in requirements):
                        #print "pass"
                        return self.pass_trough_problem(a, **kwargs)
            return f(self, *args, **kwargs)
        return wrapper
    return decorator

class Translator(object):
    def __init__(self, copy=True, **kwargs):
        self.depends = []
        self.set_copy(copy)

    def set_copy(self, value):
        if self.depends:
            for d, v in zip(self.depends, [value] + [False]*(len(self.depends)-1)):
                d.set_copy(v)
            self.copy = False
        else:
            self.copy = value
        
    def translate(self, entity, **kwargs):
        t0 = time.time()
        # print "preorder:", type(self), type(entity)
        for translator in self.depends:
            #entity._original = Translator.get_original(entity)
            #original = entity
            # 
            entity = translator.translate(entity, **kwargs)
            #entity._original = original._original
        t1 = time.time()

        # print "postorder:", type(self), type(entity)
        if isinstance(entity, actions.Action):
            result = self.translate_action(entity, **kwargs)
        elif isinstance(entity, axioms.Axiom):
            result = self.translate_axiom(entity, **kwargs)
        elif isinstance(entity, problem.Problem):
            orig_domain = Translator.get_original(entity.domain)
            result = self.translate_problem(entity, **kwargs)
            result.domain._original = orig_domain
        elif isinstance(entity, domain.Domain):
            result = self.translate_domain(entity, **kwargs)
            
        result._original = Translator.get_original(entity)
        # print "total time for %s: %.2f (this: %.2f)" % (type(self), time.time()-t0, time.time()-t1)
        return result


    @staticmethod
    def get_original(entity):
        if '_original' in entity.__dict__:
            return entity._original
        return entity

    @staticmethod
    def get_annotations(entity):
        entity = Translator.get_original(entity)
        if 'annotations' not in entity.__dict__:
            entity.annotations = TranslatorAnnotations()
        return entity.annotations

    def translate_action(self, action, domain=None):
        if self.copy:
            return action.copy(newdomain=domain)
        if domain:
            action.set_parent(domain)
        return action

    def translate_axiom(self, axiom, domain=None):
        if self.copy:
            return axiom.copy(newdomain=domain)
        if domain:
            axiom.set_parent(domain)
        return axiom

    def pass_trough_domain(self, _domain):
        if self.copy:
            return _domain.copy()
        else:
            return _domain
    
    def translate_domain(self, _domain):
        if self.copy:
            dom = _domain.copy_skeleton()
        else:
            dom = _domain

        actions = _domain.get_action_like() 
        axioms = _domain.axioms
        dom.clear_actions()
        dom.axioms = []

        for a in actions:
            dom.add_action(self.translate_action(a, dom))
        dom.axioms += [self.translate_axiom(a, dom) for a in axioms]
        dom.stratify_axioms()
        return dom

    def pass_trough_problem(self, _problem):
        if self.copy:
            domain = self.pass_trough_domain(_problem.domain)
            return problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        else:
            return _problem
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        if self.copy:
            return problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        _problem.set_parent(domain)
        return _problem

    def change_functions(self, table, change_func):
        changes = []
        for f in table:
            result = change_func(f)
            if result:
                changes.append((f,result))
        for old, new in changes:
            table.remove(old)
            table.add(new)

    def add_function(self, function, domain):
        table = domain.predicates if isinstance(function, Predicate) else domain.functions
        
        f = table.get(function.name, function.args)
        if f:
            if f.builtin != function.builtin:
                table.remove(f)
            else:
                return
            
        table.add(function)

def change_builtin_functions(table, functions=None):
    if functions is not None:
        fnew = set()
        for f in functions:
            if isinstance(f, Function):
                fnew.add(f.name)
            else:
                fnew.add(f)
        functions = fnew
        
    changes = []
    for f in table:
        if f.builtin and (functions is None or f.name in functions):
            changes.append(f)

    for f in changes:
        table.remove(f)
        pdict = {}
        newargs = []
        for a in f.args:
            if isinstance(a.type, types.ProxyType):
                a2 = types.Parameter(a.name, types.ProxyType(pdict[a.type.parameter.name]))
            else:
                a2 = types.Parameter(a.name, a.type)
            newargs.append(a2)
            pdict[a.name] = a2
        if f.__class__ == Predicate:
            table.add(f.__class__(f.name, newargs, builtin=False, function_scope=f.function_scope))
        else:
            table.add(f.__class__(f.name, newargs, f.type, builtin=False, function_scope=f.function_scope))

def get_function(lit):
    if lit.predicate in (equals, eq, assign, num_assign, equal_assign, num_equal_assign):
        if isinstance(lit.args[0], FunctionTerm):
            return lit.args[0].function
    return lit.predicate

def set_modality(lit, modality, pre_args=[], post_args=[]):
    if lit.predicate in (equals, assign, equal_assign):
        if len(modality.args) == len(pre_args) + len(post_args) + 1:
            return lit.__class__(modality, pre_args + lit.args[:1] + post_args, lit.scope, lit.negated)
        elif len(modality.args) == len(pre_args) + len(post_args) + 2:
            return lit.__class__(modality, pre_args + lit.args + post_args, lit.scope, lit.negated)
        else:
            assert False
    return lit
            
            
class ChainingTranslator(Translator):
    def __init__(self, *deps, **kwargs):
        copy = kwargs.get('copy', True)
        self.depends = deps
        self.set_copy(copy)

    def translate_problem(self, _problem):
        return self.pass_trough_problem(_problem)

    def translate_domain(self, _domain):
        return self.pass_trough_domain(_domain)
    
class IntermediateCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = []
        self.counter = 0
        self.set_copy(copy)

    def translate_domain(self, _domain):
        return self.pass_trough_domain(_domain)

    def translate_problem(self, _problem):
        p2 = Translator.translate_problem(self, _problem)

        self.counter = 0

        @visitors.replace
        def visitor(cond, parts):
            if isinstance(cond, conditions.IntermediateCondition):
                helpLitStr = "intermediate-%i-fullfilled" % self.counter
                actionName = "fullfill-intermediate-%i" % self.counter
                pred = predicates.Predicate(helpLitStr,[])
                p2.domain.predicates.add(pred)

                goalToken = conditions.LiteralCondition(pred,[], cond.scope)
                eff = effects.SimpleEffect(pred,[],cond.scope)
              
                consts = cond.get_constants()
                p2.domain.constants |= consts
                p2.domain.add(consts)

                for c in consts:
                    p2.remove_object(c)

                helpAct = actions.Action(actionName, [], cond.cond, eff, p2.domain)
                costIncrease = predicates.Term(0.0)
                helpAct.set_total_cost(costIncrease)
                p2.domain.actions.append(helpAct)

                self.counter += 1
                return goalToken

        p2.goal = p2.goal.visit(visitor)
        
        return p2

class PreferenceCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [IntermediateCompiler(**kwargs)]
        self.set_copy(copy)
        self.counter = 0
        self.costIncrease = 0
        self.prefCondScope = None

    def translate_action(self, action, domain):
        @visitors.collect
        def collect_prefs_visitor(cond, parts):
            if isinstance(cond, conditions.PreferenceCondition):
                return cond

        prefs = visitors.visit(action.precondition, collect_prefs_visitor, [])
        assert len(prefs) <= 1, "only one preference per action precondition supported"

        if not prefs:
            return Translator.translate_action(self, action, domain) 
        
        
        @visitors.copy
        def visitor1(cond, parts):
            if isinstance(cond, conditions.PreferenceCondition):
                self.costIncrease = cond.penalty
                self.prefCondScope = cond.scope
                return conditions.Conjunction([])

        action1 = action.copy(domain)
        action1.precondition = visitors.visit(action.precondition, visitor1)

        action1.name = "ignore-preferences-" + action1.name
        old_total_cost = action1.get_total_cost()
        new_cost_term = None

        if old_total_cost == None:
            new_cost_term = predicates.Term(self.costIncrease)
        else:
            new_cost_term = predicates.Term(self.costIncrease + old_total_cost.object.value)

        action1.set_total_cost(new_cost_term)
        action2 = Translator.translate_action(self, action, domain)

        @visitors.copy
        def visitor2(cond, parts):
            if isinstance(cond, conditions.PreferenceCondition):
                return cond.cond

        action2.precondition = action2.precondition.visit(visitor2)
        domain.actions.append(action2)

        return action1

    def translate_problem(self, _problem):
        import builder
        p2 = Translator.translate_problem(self, _problem)

        if 'soft_goals' not in Translator.get_annotations(_problem):
            Translator.get_annotations(_problem)['soft_goals'] = []
        
        self.counter = 0

        @visitors.replace
        def visitor(cond, parts):
            if isinstance(cond, conditions.PreferenceCondition):
                Translator.get_annotations(_problem)['soft_goals'].append(cond)
                help_lit_str = "preference-%i-ignored" % self.counter
                action_name = "ignore-preference-%i" % self.counter
                pred = predicates.Predicate(help_lit_str,[])
                p2.domain.predicates.add(pred)

                help_lit = conditions.LiteralCondition(pred,[], cond.scope)
                parts = [cond.cond, help_lit]
                goal_dis = conditions.Disjunction(parts, cond.scope)
                
                precond = help_lit.negate()
                help_act = actions.Action(action_name, [], precond, None, p2.domain)
                b = builder.Builder(help_act)
                help_act.effect = b.effect("and", (pred,),
                                           ("increase", ("virtual-cost",), cond.penalty),
                                           ("increase", ("total-cost",), 0))

                p2.domain.actions.append(help_act)

                self.counter += 1
                return goal_dis

        p2.goal = p2.goal.visit(visitor)
        return p2
        
    
class ADLCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [ModalPredicateCompiler(**kwargs), ObjectFluentCompiler(**kwargs), CompositeTypeCompiler(**kwargs), PreferenceCompiler(**kwargs)]
        self.set_copy(copy)

    @staticmethod
    def condition_visitor(cond, parts):
        if isinstance(cond, conditions.Truth):
            return conditions.Conjunction([])
        return visitors.flatten(cond, parts)
        
    def translate_action(self, action, domain=None):
        a2 = Translator.translate_action(self, action, domain)

        if "action-costs" in domain.requirements:
            if a2.get_total_cost() == None:
                a2.set_total_cost(predicates.Term(1.0))
                            
        a2.precondition = visitors.visit(a2.precondition, ADLCompiler.condition_visitor)
        a2.replan = visitors.visit(a2.replan, ADLCompiler.condition_visitor)

        return a2

    def translate_axiom(self, axiom, domain=None):
        a2 = Translator.translate_axiom(self, axiom, domain)
        a2.condition = visitors.visit(a2.condition, ADLCompiler.condition_visitor)
        return a2

    # def translate_domain(self, _domain):
    #     dom = Translator.translate_domain(self, _domain)
    #     dom.requirements.discard('numeric-fluents')
    #     return dom
    
    def translate_problem(self, _problem):
        p2 = Translator.translate_problem(self, _problem)

        p2.init = [i for i in p2.init if not i.negated]
        p2.goal = visitors.visit(p2.goal, ADLCompiler.condition_visitor)
        if "action-costs" in _problem.domain.requirements:
            p2.optimization = "minimize"
            p2.opt_func = FunctionTerm(total_cost,[])

        #make sure that the goal is in the form FD likes (exactly one "and" at the top level)
        # if not isinstance(p2.goal, conditions.Conjunction):
        #     p2.goal = conditions.Conjunction([p2.goal])
        # else:
        #     newparts = []
        #     open_parts = p2.goal.parts[:]
        #     while open_parts:
        #         part = open_parts.pop(0)
        #         if isinstance(part, conditions.Conjunction):
        #             open_parts += part.parts
        #         else:
        #             newparts.append(part)
        #     p2.goal = conditions.Conjunction(newparts)
            
        return p2

class CompositeTypeCompiler(Translator):
    @staticmethod
    def replacement_type(typ, typedict):
        if not isinstance(typ, types.CompositeType):
            return typ
        
        supertypes = set(typedict.itervalues())
        for t in typ.types:
            supertypes &= t.supertypes
            
        for t in typedict.itervalues():
            if t not in typ.types:
                supertypes -= t.supertypes

        if supertypes:
            return iter(supertypes).next()
        newtype = types.Type("either_%s" % "_".join(t.name for t in typ.types))
        for t in typ.types:
            t.supertypes.add(newtype)
        typedict[newtype.name] = newtype
        return newtype

    @staticmethod
    def replace_args(elem):
        new_args = []
        for arg in elem.args:
            arg = types.Parameter(arg.name, CompositeTypeCompiler.replacement_type(arg.type, elem.types))
            new_args.append(arg)
        elem.args = elem.copy_args(new_args)
    
    @staticmethod
    def type_visitor(elem, results):
        if isinstance(elem, conditions.QuantifiedCondition):
            CompositeTypeCompiler.replace_args(elem)
            elem.condition.set_scope(elem)
        if isinstance(elem, effects.UniversalEffect):
            CompositeTypeCompiler.replace_args(elem)
            elem.effect.set_scope(elem)
        elif isinstance(elem, effects.ConditionalEffect):
            elem.condition.visit(CompositeTypeCompiler.type_visitor)

    def translate_action(self, action, domain=None):
        a2 = Translator.translate_action(self, action, domain)
        CompositeTypeCompiler.replace_args(a2)
        
        visitors.visit(a2.precondition, CompositeTypeCompiler.type_visitor)
        visitors.visit(a2.replan, CompositeTypeCompiler.type_visitor)
        visitors.visit(a2.effect, CompositeTypeCompiler.type_visitor)
        return a2

    def translate_axiom(self, axiom, domain=None):
        a2 = Translator.translate_axiom(self, axiom, domain)
        CompositeTypeCompiler.replace_args(a2)
        
        a2.condition.visit(CompositeTypeCompiler.type_visitor)
        return a2
        
    def translate_domain(self, _domain):
        dom = _domain.copy_skeleton()

        dom.predicates = scope.FunctionTable()
        for func in _domain.predicates:
            args = [types.Parameter(a.name, CompositeTypeCompiler.replacement_type(a.type, dom.types)) for a in func.args]
            dom.predicates.add(predicates.Predicate(func.name, args, func.builtin, func.function_scope))
                                                    
        dom.functions = scope.FunctionTable()
        for func in _domain.functions:
            args = [types.Parameter(a.name, CompositeTypeCompiler.replacement_type(a.type, dom.types)) for a in func.args]
            ftype = CompositeTypeCompiler.replacement_type(func.type, dom.types)
            dom.functions.add(predicates.Function(func.name, args, ftype, func.builtin, func.function_scope))
        
        dom.clear_actions()
        for a in _domain.get_action_like():
            dom.add_action(self.translate_action(a, dom))
            
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()

        return dom
    

class ObjectFluentNormalizer(Translator):
    def create_param(self, name, type, names):
        pname = name
        i = 1
        while pname in names:
            pname = "%s%d" % (name, i)
            i += 1
        return types.Parameter(pname, type )

    def translate_literal(self, lit, termdict):
        names = set(p.name for p in termdict.itervalues())
        
        def term_visitor(term, results):
            if isinstance(term, FunctionVariableTerm) or term.get_type().equal_or_subtype_of(t_number):
                return term
            if isinstance(term, FunctionTerm):
                args = []
                for t, farg in zip(results, term.function.args):
                    if isinstance(farg.type, types.FunctionType):
                        args.append(t)
                    elif isinstance(t, FunctionTerm):
                        if t in termdict:
                            args.append(Term(termdict[t]))
                        else:
                            param = self.create_param("?%s" % t.function.type.name, t.function.type, names)
                            names.add(param.name)
                            termdict[t] = param
                            args.append(Term(param))
                    else:
                        args.append(t)
                return FunctionTerm(term.function, args)
            else:
                return term

        args = lit.args
        pargs = lit.predicate.args
        new_args = []
        if lit.predicate in numeric_comparators + [equals]:
            new_args.append(lit.args[0].visit(term_visitor))
            args = args[1:]
            pargs = lit.predicate.args[1:]
            
        for arg, parg in zip(args, pargs):
            t = arg.visit(term_visitor)
            if isinstance(parg.type, types.FunctionType) or parg.is_instance_of(t_number):
                new_args.append(t)
            elif isinstance(t, FunctionTerm):
                if t in termdict:
                    new_args.append(Term(termdict[t]))
                else:
                    param = self.create_param("?%s" % t.function.type.name, t.function.type, names)
                    names.add(param.name)
                    termdict[t] = param
                    new_args.append(Term(param))
            else:
                new_args.append(t)

        return lit.new_literal(args=new_args, scope=None)

    def dependent_terms(self, termdict, args):
        dep = set(a for a in args)
        subdict = {}
        done = False
        while not done:
            done = True
            for t, param in termdict.iteritems():
                if t in subdict:
                    continue
                
                args = set(t.visit(predicates.collect_args_visitor))
                if dep & args:
                    subdict[t] = param
                    dep.add(param)
                    done = False
        return subdict

    def create_temp_scope(self, scope, termdict, domain):
        if not scope:
            result = Scope([], domain)
        else:
            result = Scope([a for a in scope.itervalues()], scope.parent)
        result.add(termdict.values())
        return result

    def translate_condition(self, cond, termdict, domain):
        @visitors.copy
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate in numeric_comparators + [equals]:
                    if cond.predicate in (equals, eq):
                        new_pred = cond.predicate
                    elif cond.predicate == gt:
                        new_pred = lt
                    elif cond.predicate == lt:
                        new_pred = gt
                    elif cond.predicate == ge:
                        new_pred = le
                    elif cond.predicate == le:
                        new_pred = ge
                        
                    if isinstance(cond.args[0], (VariableTerm, ConstantTerm)) and isinstance(cond.args[1], FunctionTerm):
                        cond = cond.new_literal(predicate=new_pred, args= [cond.args[1], cond.args[0]])
                    elif isinstance(cond.args[0], (VariableTerm, ConstantTerm)) and isinstance(cond.args[1], (VariableTerm, ConstantTerm)):
                        return cond
                    if isinstance(cond.args[1], FunctionTerm) and cond.args[0] in termdict and cond.args[1] not in termdict:
                        cond = cond.new_literal(predicate=new_pred, args=[cond.args[1], cond.args[0]])

                return self.translate_literal(cond, termdict)
            
            elif isinstance(cond, conditions.QuantifiedCondition):
                #Terms that contain quantified variables must be handled inside the quantifier
                subdict = self.dependent_terms(termdict, cond.args)
                temp_scope = self.create_temp_scope(cond, termdict, domain)
                if not subdict:
                    return cond.copy(new_parts=results, new_scope=temp_scope)
                
                args = subdict.values()
                if isinstance(cond, conditions.ExistentialCondition):
                    #just add the new terms to the Ex. Quantifier
                    result = conditions.ExistentialCondition(cond.args+args, conditions.Conjunction.new(results[0]), temp_scope)
                    conj = result.condition
                elif isinstance(cond, conditions.UniversalCondition):
                    #create an Ex. Quantifier inside the Univ. Quantifier
                    result = conditions.UniversalCondition(cond.args, None, temp_scope)
                    ex = conditions.ExistentialCondition(args, conditions.Conjunction.new(results[0]), result)
                    result.condition = ex
                    conj = ex.condition
                for t, arg in subdict.iteritems():
                    conj.parts.append(conditions.LiteralCondition(equals, [t, Term(arg)]))
                    del termdict[t] #those terms are limited to the current scope. Remove from global termdict
                return result

        return visitors.visit(cond, visitor)
        
    def translate_effect(self, eff, termdict, domain):
        @visitors.copy
        def visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return self.translate_literal(eff, termdict)
            elif isinstance(eff, effects.ConditionalEffect):
                temp_scope = self.create_temp_scope(eff.get_scope(), termdict, domain)
                cond = self.translate_condition(eff.condition, termdict, domain)
                return effects.ConditionalEffect(cond, results[0], temp_scope)
            elif isinstance(eff, effects.UniversalEffect):
                #Terms that contain quantified variables must be handled inside the quantifier
                temp_scope = self.create_temp_scope(eff, termdict, domain)
                subdict = self.dependent_terms(termdict, eff.args)
                if not subdict:
                    return eff.copy(new_parts=results, new_scope=temp_scope)
                
                #Add the terms to the ex. quantifier and add a conditional effect inside
                args = subdict.values()
                result = effects.UniversalEffect(eff.args+args, None, temp_scope)
                if isinstance(results[0], effects.ConditionalEffect):
                    result.effect = results[0]
                    result.effect.condition = conditions.Conjunction.new(result.effect.condition)
                    cond = result.effect.condition
                else:
                    cond = conditions.Conjunction([])
                    result.effect = effects.ConditionalEffect(cond, results[0], scope=result)
                for t, arg in subdict.iteritems():
                    cond.parts.append(conditions.LiteralCondition(equals, [t, Term(arg)]))
                    del termdict[t] #those terms are limited to the current scope. Remove from global termdict
                return result

        return visitors.visit(eff, visitor)
        
    def translate_action(self, action, domain=None):
        if domain is None:
            domain = action.parent

        action.set_lazy_mode(params=scope.UNKNOWN_OBJECT_IGNORE)
        termdict = {}
        pre = self.translate_condition(action.precondition, termdict, domain)
        replan = self.translate_condition(action.replan, termdict, domain)
        effect = self.translate_effect(action.effect, termdict, domain)
        action.set_lazy_mode(params=scope.UNKNOWN_OBJECT_ERROR)

        add_args = []
        if termdict:
            pre = conditions.Conjunction.new(pre)
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(equals, [term, Term(param)]))
                add_args.append(param)
            # if action.precondition is None:
            #     print pre.pddl_str()

        a2 = action.copy_skeleton(domain)
        a2.add(add_args)
        a2.args += add_args

        if pre:
            pre.set_scope(a2)
            a2.precondition = pre
        if replan:
            replan.set_scope(a2)
            a2.replan = replan
        if effect:
            effect.set_scope(a2)
            a2.effect = effect
        return a2

    def translate_axiom(self, axiom, domain=None):
        if domain is None:
            domain = action.parent

        termdict = {}
        pre = self.translate_condition(axiom.condition, termdict, domain)

        a2 = axioms.Axiom(domain.predicates.get(axiom.predicate.name, axiom.args), [types.Parameter(p.name, p.type) for p in axiom.args], None, domain)
        
        if termdict:
            args = []
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(equals, [term, Term(param)]))
                args.append(param)
            a2.condition = conditions.ExistentialCondition(args, pre, a2)
        else:
            a2.condition = pre.copy(a2)

        return a2

    @requires('object-fluents', 'fluents')
    def translate_domain(self, _domain):
        return Translator.translate_domain(self, _domain)
    
    @requires('object-fluents', 'fluents')
    def translate_problem(self, _problem):
        return Translator.translate_problem(self, _problem)

class ObjectFluentCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [ModalPredicateCompiler(**kwargs), ObjectFluentNormalizer(**kwargs)]
        self.set_copy(copy)
        
    def translate_condition(self, cond, scope):
        if cond is None:
            return None, []
        
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == equals:
                    if  isinstance(cond.args[0], (VariableTerm, ConstantTerm)) and isinstance(cond.args[1], (VariableTerm, ConstantTerm)):
                        return cond, []
                    assert isinstance(cond.args[0], FunctionTerm) and isinstance(cond.args[1], (VariableTerm, ConstantTerm))
                    new_pred = scope.predicates.get(cond.args[0].function.name, cond.args[0].args + cond.args[-1:])
                    if not new_pred:
                        err =  "%s, %s" % (cond.args[0].function.name, ", ".join(str(a.object) for a in cond.args[0].args + cond.args[-1:]))
                        err += cond.pddl_str()
                        assert False, "invalid predicate: %s" % err
                    
                    if cond.negated:
                        return conditions.LiteralCondition(new_pred, cond.args[0].args[:] + [cond.args[1]], negated=True), []
                    return conditions.LiteralCondition(new_pred, cond.args[0].args[:] + [cond.args[1]], negated=cond.negated), [(cond.args[0], cond.args[1])]
                    
                return cond, []
            else:
                facts = []
                parts = []
                for p,f in results:
                    parts.append(p)
                    facts += f
                return cond.copy(new_parts=parts), facts
        result, facts = cond.visit(visitor)

        return result.copy(new_scope=scope), facts
        
    def translate_effect(self, eff, read_facts, scope):
        if eff is None:
            return None
        
        previous_values = defaultdict(set)
        for term, value in read_facts:
            previous_values[term].add(value)

        @visitors.copy
        def effectsVisitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate in (assign, change):
                    assert isinstance(eff.args[0], FunctionTerm) and isinstance(eff.args[1], (VariableTerm, ConstantTerm))
                    term = eff.args[0]
                    value = eff.args[1]
                    new_pred = scope.predicates.get(term.function.name, term.args + eff.args[-1:])
                    effs = []
                    if term in previous_values:
                        for val in previous_values[term]:
                            del_eff = effects.SimpleEffect(new_pred, term.args[:] + [val], negated=True)
                            if isinstance(val, ConstantTerm) and isinstance(value, ConstantTerm):
                                if val != value:
                                    effs.append(del_eff)
                            else:
                                #I forgot the reason for this... probably a FD bug
                                #del_cond = conditions.LiteralCondition(equals, [val, value], negated=True)
                                #effs.append(effects.ConditionalEffect(del_cond, del_eff))
                                effs.append(del_eff)
                        effs.append(effects.SimpleEffect(new_pred, term.args[:] + [value]))
                    elif term.function.type == t_boolean:
                        if eff.args[1] == TRUE:
                            effs.append(effects.SimpleEffect(new_pred, term.args[:] + [FALSE], negated=True))
                        else:
                            effs.append(effects.SimpleEffect(new_pred, term.args[:] + [TRUE], negated=True))
                        effs.append(effects.SimpleEffect(new_pred, term.args[:] + [value]))
                    else:
                        param = types.Parameter("?oldval", term.function.type)
                        condition = conditions.LiteralCondition(equals, [Term(param), value], negated=True)
                        negeffect = effects.SimpleEffect(new_pred, term.args[:] + [Term(param)], negated=True)
                        ceffect = effects.ConditionalEffect(condition, negeffect)
                        #effs = [effects.UniversalEffect([param], ceffect, None), effects.SimpleEffect(new_pred, term.args[:] + [value])]
                        effs = [effects.SimpleEffect(new_pred, term.args[:] + [value])]
                    return effects.ConjunctiveEffect(effs)
            elif isinstance(eff, effects.ConditionalEffect):
                #TODO: take the read facts from a conditional effect into account
                temp_scope = Scope([], eff.scope)
                temp_scope.predicates = scope.predicates
                cond, _ = self.translate_condition(eff.condition, temp_scope)
                return effects.ConditionalEffect(cond, results[0])
                                                

        eff.set_scope(scope)
        result = eff.visit(effectsVisitor)
        result.set_scope(scope)
        return result.visit(visitors.flatten)
        
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        #a2 = actions.Action(action.name, [types.Parameter(p.name, p.type) for p in action.args], None, None, domain)
        a2 = action.copy_skeleton(domain)
        a2.precondition, facts = self.translate_condition(action.precondition, a2)
        a2.replan, rfacts = self.translate_condition(action.replan, a2)
        facts += rfacts

        if action.effect:
            a2.effect = action.effect.copy(new_scope=a2)
            #print a2.effect.pddl_str()
            a2.effect = self.translate_effect(action.effect, facts, a2)

        return a2
            
    def translate_axiom(self, axiom, domain=None):
        assert domain is not None

        a2 = axioms.Axiom(domain.predicates.get(axiom.predicate.name, axiom.args), [types.Parameter(p.name, p.type) for p in axiom.args], None, domain)
        a2.condition, _ = self.translate_condition(axiom.condition, a2)
        return a2

    @requires('object-fluents', 'fluents')
    def translate_domain(self, _domain):
        predicates = []
        functions = []
        for f in _domain.functions:
            if not f.type.equal_or_subtype_of(t_number):
                predicates.append(Predicate(f.name, [types.Parameter(p.name, p.type) for p in f.args] + [types.Parameter("?value", f.type)]))
            else:
                functions.append(f)

        functions = scope.FunctionTable(functions)
        predicates = scope.FunctionTable(predicates)
        for p in _domain.predicates:
            predicates.add(p)

        dom = _domain.copy_skeleton()
        dom.requirements.discard('object-fluents')
        if 'fluents' in dom.requirements:
            dom.requirements.discard('fluents')
            dom.requirements.add('numeric-fluents')
            
        dom.functions = functions
        dom.predicates = predicates
            
        dom.clear_actions()
        for a in _domain.get_action_like():
            dom.add_action(self.translate_action(a, dom))
            
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        return dom
    
    @requires('object-fluents', 'fluents')
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)
        
        p2.goal,_ = self.translate_condition(_problem.goal, p2)

        @visitors.copy
        def eff_visitor(lit, results):
            if isinstance(lit, Literal):
                if lit.predicate in assignment_ops:
                    if lit.args[0].function.type.equal_or_subtype_of(t_number):
                        return lit
                    new_pred = domain.predicates.get(lit.args[0].function.name, lit.args[0].args + lit.args[-1:])
                    return lit.__class__(new_pred, lit.args[0].args[:] + [lit.args[1]], p2)

        for i in _problem.init:
            lit = i.visit(eff_visitor)
            if lit:
                lit.set_scope(p2)
                p2.init.append(lit)
        
        return p2

class NumericConstantCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [MAPLCompiler(**kwargs), ObjectFluentNormalizer(**kwargs)]
        self.set_copy(copy)
        self.numericFluentToPredicateDict = dict()

    def translate_problem(self, _problem):
        from itertools import product
        import state
        
        _problem = Translator.translate_problem(self,_problem)

        s = state.State.from_problem(_problem)
        for k,v in self.numericFluentToPredicateDict.iteritems():
            args0, args1, pred, cond, literalParams, action = v
            combinations = product(*map(lambda a: list(_problem.get_all_objects(a.type)), pred.args))

            for c in combinations:
                action.instantiate(dict(zip(literalParams,c)),parent=_problem)
                if s.evaluate_literal(cond):
                    lit = predicates.Literal(pred,c,_problem)
                    _problem.init.append(lit)
                    
                action.uninstantiate()

        return _problem
    
    def translate_action(self, action, domain=None, new_args=None):
        assert domain is not None
        
        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == gt or cond.predicate == lt or cond.predicate == ge or cond.predicate == le or (cond.predicate ==eq and cond.predicate.args[0].type == t_number):
                    if cond.predicate == gt:
                        name = "gt_"
                    elif cond.predicate == lt:
                        name = "lt_"
                    elif cond.predicate == ge:
                        name = "ge_"
                    elif cond.predicate == le:
                        name = "le_"
                    else:
                        assert(cond.predicate ==eq)
                        name = "eq_"
                    literalParams = []
                    predParams = []
                    assert(len(cond.args) == 2)  
                    if isinstance(cond.args[0],predicates.ConstantTerm):
                        #print(cond.args[0].object)
                        name += str(cond.args[0].object.value)
                    else:
                       assert isinstance(cond.args[0],predicates.FunctionTerm)
                       name += cond.args[0].function.name
                       literalParams.extend(cond.args[0].args)
                       predParams.extend(cond.args[0].function.args)
                       
                    name += "__"
                    
                    if isinstance(cond.args[1],predicates.ConstantTerm):
                        #print(cond.args[1].object)
                        name += str(cond.args[1].object.value)
                    else:
                       assert isinstance(cond.args[1],predicates.FunctionTerm)
                       name += cond.args[1].function.name
                       literalParams.extend(cond.args[1].args)
                       predParams.extend(cond.args[1].function.args)

                    
                    if name not in self.numericFluentToPredicateDict:
                        res = Predicate(name,predParams)
                        domain.predicates.add(res)
                        self.numericFluentToPredicateDict[name] = [cond.args[0],cond.args[1],res,cond,literalParams,action]
                    else:
                        res = domain.predicates.get(name,predParams)
                    return conditions.LiteralCondition(res,literalParams)
 
        action.precondition = visitors.visit(action.precondition, cond_visitor)
        return action

    def translate_axiom(self, axiom, domain=None, new_args=None, new_pred=None):
        assert domain is not None
        
        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == gt or cond.predicate == lt or cond.predicate == ge or cond.predicate == le or (cond.predicate ==eq and cond.predicate.args[0].type == t_number):
                    if cond.predicate == gt:
                        name = "gt_"
                    elif cond.predicate == lt:
                        name = "lt_"
                    elif cond.predicate == ge:
                        name = "ge_"
                    elif cond.predicate == le:
                        name = "le_"
                    else:
                        assert(cond.predicate ==eq)
                        name = "eq_"
                    literalParams = []
                    predParams = []
                    assert(len(cond.args) == 2)  
                    if isinstance(cond.args[0],predicates.ConstantTerm):
                        #print(cond.args[0].object)
                        name += str(cond.args[0].object.value)
                    else:
                       assert isinstance(cond.args[0],predicates.FunctionTerm)
                       name += cond.args[0].function.name
                       literalParams.extend(cond.args[0].args)
                       predParams.extend(cond.args[0].function.args)
                       
                    name += "__"
                    
                    if isinstance(cond.args[1],predicates.ConstantTerm):
                        #print(cond.args[1].object)
                        name += str(cond.args[1].object.value)
                    else:
                       assert isinstance(cond.args[1],predicates.FunctionTerm)
                       name += cond.args[1].function.name
                       literalParams.extend(cond.args[1].args)
                       predParams.extend(cond.args[1].function.args)

                    
                    if name not in self.numericFluentToPredicateDict:
                        res = Predicate(name,predParams)
                        domain.predicates.add(res)
                        self.numericFluentToPredicateDict[name] = [cond.args[0],cond.args[1],res,cond,literalParams,axiom]
                    else:
                        res = domain.predicates.get(name,predParams)
                    return conditions.LiteralCondition(res,literalParams)
 
        axiom.condition = visitors.visit(axiom.condition, cond_visitor)
        return axiom
      
class ModalPredicateCompiler(Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [MAPLCompiler(**kwargs)]
        self.set_copy(copy)
        self.used_functions = defaultdict(set)
        
    def compile_modal_args(self, args, functions):
        func_arg = None
        funcs = []
        pref = []
        suff = []

        for a in args:
            if isinstance(a.type, types.FunctionType):
                assert func_arg is None, "Only one functional parameter currently possible."
                func_arg = a
                for func in functions:
                    if not func.type.equal_or_subtype_of(a.type.type):
                        continue
                    funcs.append(func)
            else:
                # if isinstance(a.type, types.ProxyType):
                #     a = types.Parameter(a.name, a.type)
                #     proxy_args.add(a)

                if func_arg:
                    suff.append(a)
                else:
                    pref.append(a)

        compiled = []
        for f in funcs:
            new_args = []
            for a in pref + f.args + suff:
                if isinstance(a.type, types.ProxyType):
                    type = f.type
                else:
                    type = a.type
                new_args.append(types.Parameter(a.name, type))
                
            compiled.append((f, new_args))

        return func_arg, compiled

    def translate_literal(self, literal, scope):
        import durative

        if literal.predicate in durative.default_predicates + numeric_ops + assignment_ops:
            return literal
        
        args = []
        name_elems = [literal.predicate.name]
        has_functionals = False
        for arg, parg in zip(literal.args, literal.predicate.args):
            if isinstance(parg.type, types.FunctionType):
                name_elems.append(arg.function.name)
                args += arg.args
                has_functionals = True
                self.used_functions[literal.predicate].add(arg.function)
            else:
                args.append(arg)
        if not has_functionals:
            return literal

        pred = scope.predicates.get("-".join(name_elems), args)
        if not pred:
            return None
        result = literal.new_literal(predicate=pred, args=args)
        return result#.copy_instance()#FIXME: is this copy required?
                
    def translate_action(self, action, domain=None, new_args=None):
        assert domain is not None

        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)

        @visitors.replace
        def eff_visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return self.translate_literal(eff, domain)
            elif isinstance(eff, effects.ConditionalEffect):
                cond = eff.condition.visit(cond_visitor)
                return effects.ConditionalEffect(cond, results[0])
            
        if new_args:
            args = new_args
            copy = True
        else:
            args = [types.Parameter(p.name, p.type) for p in action.args]
            copy = self.copy

        if copy:
            a2 = action.copy_skeleton(domain) # TODO: new_args is being ignored, which is a bug
            if action.precondition:
                a2.precondition = action.precondition.copy(copy_instance=True).visit(cond_visitor)
                a2.precondition.set_scope(a2)
            if action.replan:
                a2.replan = action.replan.copy(copy_instance=True).visit(cond_visitor)
                a2.replan.set_scope(a2)
            a2.effect = visitors.visit(action.effect, eff_visitor)
            return a2
        else:
            action.precondition = visitors.visit(action.precondition, cond_visitor)
            action.replan = visitors.visit(action.replan, cond_visitor)
            action.effect = visitors.visit(action.effect, eff_visitor)
            return action
            
            
    def translate_axiom(self, axiom, domain=None, new_args=None, new_pred=None):
        assert domain is not None

        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)
        
        if new_args is not None:
            args = new_args
            copy = True
        else:
            args = [types.Parameter(p.name, p.type) for p in axiom.args]
            copy = self.copy

        if new_pred is not None:
            pred = new_pred
        else:
            pred = domain.predicates.get(axiom.predicate.name, args)

        if copy:
            a2 = axioms.Axiom(pred, args, None, domain)
            a2.condition = axiom.condition.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
            a2.condition.set_scope(a2)
            return a2
        else:
            axiom.condition = visitors.visit(axiom.condition, cond_visitor)
            return axiom

        
    @requires('modal-predicates')
    def translate_domain(self, _domain):
        import durative
        
        modal = []
        nonmodal = []
        for pred in _domain.predicates:
            if pred not in durative.default_predicates + numeric_ops + assignment_ops and any(isinstance(a.type, types.FunctionType) for a in pred.args):
                modal.append(pred)
            else:
                nonmodal.append(pred)
        if not modal:
            return _domain.copy()

        funcs = [f for f in _domain.functions if not f.builtin]

        new_pred_dict = {}
        new_preds = []
        for pred in modal:
            func_arg, compiled = self.compile_modal_args(pred.args, funcs)
            for f, args in compiled:
                new_p = Predicate("%s-%s" % (pred.name, f.name), args)
                new_pred_dict[(pred, f)] = new_p
                new_preds.append(new_p)

        nonmodal += new_preds

        dom = _domain.copy_skeleton()
        dom.requirements.discard("modal-predicates")
        dom.predicates = scope.FunctionTable(nonmodal)
        
        for ac in _domain.get_action_like():
            func_arg, compiled = self.compile_modal_args(ac.args, funcs)
            if not func_arg:
                dom.add_action(self.translate_action(ac, dom))
            else:
                for f, args in compiled:
                    ac.instantiate({func_arg : FunctionTerm(f, [Term(a) for a in f.args])})
                    a2 = self.translate_action(ac, dom, args)
                    ac.uninstantiate()
                    a2.name = "%s-%s" % (a2.name, f.name)
                    dom.add_action(a2)

        #Nonmodal axioms
        for ax in _domain.axioms:
            func_arg, compiled = self.compile_modal_args(ax.args, [])
            if not func_arg:
                dom.axioms.append(self.translate_axiom(ax, dom))

        prev_used = defaultdict(set)
        while True:
            added = False
            for ax in _domain.axioms:
                used_functions = self.used_functions[ax.predicate] - prev_used[ax.predicate]
                func_arg, compiled = self.compile_modal_args(ax.args, used_functions)
                # print ax.predicate, map(str,used_functions)
                if not func_arg:
                    continue
                for f, args in compiled:
                    added = True
                    ax.instantiate({func_arg : FunctionTerm(f, [Term(a) for a in f.args])})
                    new_pred = dom.predicates.get("%s-%s" % (ax.predicate.name, f.name), args)
                    a2 = self.translate_axiom(ax, dom, args, new_pred)
                    a2.copy()
                    ax.uninstantiate()
                    dom.axioms.append(a2)
            for k,v in self.used_functions.iteritems():
                prev_used[k] |= v
            self.used_functions.clear()
            
            if not added:
                break
            
        for (p, f), compiled in new_pred_dict.iteritems():
            if f not in prev_used[p]:
                dom.predicates.remove(compiled)
            
        dom.stratify_axioms()
        dom.name2action = None

        return dom
    
    @requires('modal-predicates')
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)

        @visitors.copy
        def lit_visitor(cond, results):
            if isinstance(cond, predicates.Literal):
                res = self.translate_literal(cond, p2)
                if not res:
                    return False
                return res

        if _problem.goal:
            p2.goal = _problem.goal.visit(lit_visitor)

        for i in _problem.init:
            res = i.visit(lit_visitor)
            if res:
                p2.init.append(res)
        
        return p2

class RemoveTimeCompiler(Translator):
    def translate_action(self, action, domain=None):
        import durative, mapl

        if domain is None:
            domain = action.domain

        if not isinstance(action, durative.DurativeAction):
            return Translator.translate_action(self, action, domain)

        duration_term = action.duration[0].term

        @visitors.copy
        def visitor(elem, results):
            if isinstance(elem, durative.TimedCondition):
                return results[0]
            if isinstance(elem, durative.TimedEffect):
                return effects.SimpleEffect(elem.predicate, [a.copy_instance() for a in elem.args], None, elem.negated)
            if isinstance(elem, effects.ConditionalEffect):
                cond = elem.condition.visit(visitor)
                return effects.ConditionalEffect(cond, results[0])
            elif isinstance(elem, effects.SimpleEffect) and elem.predicate in (change, num_change):
                if elem.predicate == change:
                    pred = assign
                else:
                    pred = num_assign
                return effects.SimpleEffect(pred, [a.copy_instance() for a in elem.args], None, elem.negated)
                
        
        if not isinstance(action, durative.DurativeAction):
            return action.copy(newdomain=domain)
        
        if isinstance(action, mapl.MAPLAction):
            vars = [a for a in action.vars if a.name != "?duration"]
            a2 = mapl.MAPLAction(action.name, action.agents, action.params, vars, None, None, None, None, domain)
            a2.sensors = [s.copy(a2) for s in action.sensors]
        else:
            args = [a for a in action.args if a.name != "?duration"]
            a2 = actions.Action(action.name, args, None, None, domain)

        action.instantiate({'?duration' : duration_term})
        action.set_total_cost(duration_term)
        
        a2.precondition = visitors.visit(action.precondition, visitor)
        a2.replan = visitors.visit(action.replan, visitor)
        a2.effect = visitors.visit(action.effect, visitor)

        action.uninstantiate()
        
        if a2.precondition:
            a2.precondition.set_scope(a2)
        if a2.replan:
            a2.replan.set_scope(a2)
        if a2.effect:
            a2.effect.set_scope(a2)

        return a2

    @requires('durative-actions')
    def translate_domain(self, domain):
        dom = Translator.translate_domain(self, domain)
        dom.requirements.discard('durative-actions')
        dom.requirements.add('action-costs')
        return dom

    @requires('durative-actions')
    def translate_problem(self, problem):
        p2 = Translator.translate_problem(self, problem)
        p2.optimization = "minimize"
        p2.opt_func = FunctionTerm(total_cost,[])
        return p2
    

class MAPLCompiler(Translator):
    def __init__(self, remove_replan=False, **kwargs):
        Translator.__init__(self)
        self.remove_replan = remove_replan
        
    def translate_action(self, action, domain=None, cond_keffs=False):
        import mapl, durative
        assert domain is not None

        def visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate in (mapl.update, mapl.update_fail):
                    return None
                if utils.is_functional(eff) and eff.args[0].function == mapl.failure_cost:
                    return None
                if eff.predicate == mapl.knowledge:
                    e2 = eff.copy()
                    e2.predicate = mapl.direct_knowledge
                    return e2
                else:
                    return eff.copy()

            filtered_results = filter(None, results)
            if not filtered_results:
                return None
            return eff.copy(new_parts = filtered_results)
                
        a2 = action.copy_skeleton(domain)
        if isinstance(action, mapl.MAPLDurativeAction):
            a2.__class__ = durative.DurativeAction
        elif isinstance(action, mapl.MAPLAction):
            a2.__class__ = actions.Action
        
        if action.precondition:
            a2.precondition = action.precondition.copy(new_scope=a2)
        if action.replan:
            if self.remove_replan:
                parts = [a2.precondition]
                parts.append(action.replan.copy(new_scope=a2))
                a2.precondition = conditions.Conjunction(parts)
            else:
                a2.replan = action.replan.copy(new_scope=a2)
                
        if action.effect:
            a2.effect = action.effect.visit(visitor)
            if a2.effect is None:
                a2.effect = effects.ConjunctiveEffect([])
            a2.effect.set_scope(a2)

        if isinstance(action, mapl.MAPLAction) and action.sensors:
            # commit_cond = action.commit_condition().copy(new_scope=a2)
            # a2.precondition = conditions.Conjunction.new(a2.precondition)
            # a2.precondition.parts.append(commit_cond)

            if cond_keffs:
                keff = action.conditional_knowledge_effect().copy(new_scope=a2)
            else:
                keff = action.knowledge_effect().copy(new_scope=a2)
                
            if a2.effect:
                keff.parts.insert(0, a2.effect)
            a2.effect = keff

        return a2

    @requires('mapl')
    def translate_domain(self, _domain):
        import mapl

        dom = _domain.copy_skeleton()
        dom.requirements.discard("mapl")
        dom.constants.discard(types.UNKNOWN)

        change_builtin_functions(dom.predicates, mapl.modal_predicates)

        
        dom.clear_actions()
        cond_keffs = Translator.get_annotations(_domain).get('has_commit_actions', False)
        for a in _domain.get_action_like():
            if a in _domain.init_rules:
                continue
            dom.add_action(self.translate_action(a, dom, cond_keffs=cond_keffs))

        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        return dom

    @requires('mapl')
    def translate_problem(self, _problem):
        p2 = Translator.translate_problem(self, _problem)

        oldinit = p2.init
        p2.init = []
        for i in oldinit:
            #determinise probabilistic init conditions
            if isinstance(i, effects.ProbabilisticEffect):
                p2.init.append(i)
            elif not i.args or i.args[-1] != builtin.UNKNOWN:
                p2.init.append(i)

        return p2


#
# Create default modules
#

domain.ModuleDescription.create_module("object-fluents", default_compiler=ObjectFluentCompiler)
domain.ModuleDescription.create_module("modal-predicates", default_compiler=ModalPredicateCompiler)
domain.ModuleDescription.create_module("preferences", default_compiler=PreferenceCompiler)
domain.ModuleDescription.create_module("numeric-fluents", default_compiler=NumericConstantCompiler)
