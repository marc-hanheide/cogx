from collections import defaultdict
import itertools, time

import predicates, conditions, effects, actions, axioms, domain, problem
import visitors
import mapltypes as types
import builtin
from builtin import *
from predicates import *
    

class Translator(object):
    def __init__(self, **kwargs):
        self.depends = []
    def translate(self, entity, **kwargs):
        t0 = time.time()
        for translator in self.depends:
            entity = translator.translate(entity)
        #print "total time for %s: %f" % (type(self), time.time()-t0)
            
        if isinstance(entity, actions.Action):
            return self.translate_action(entity, **kwargs)
        elif isinstance(entity, axioms.Axiom):
            return self.translate_axiom(entity, **kwargs)
        elif isinstance(entity, problem.Problem):
            return self.translate_problem(entity, **kwargs)
        elif isinstance(entity, domain.Domain):
            return self.translate_domain(entity, **kwargs)

    def translate_action(self, action, domain=None):
        return action.copy(newdomain=domain)

    def translate_axiom(self, axiom, domain=None):
        return axiom.copy(newdomain=domain)
    
    def translate_domain(self, _domain):
        dom = domain.Domain(_domain.name, _domain.types.copy(), set(_domain.constants), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        return problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
    
class ADLCompiler(Translator):
    def __init__(self, **kwargs):
        self.depends = [ModalPredicateCompiler(**kwargs), ObjectFluentCompiler(**kwargs), CompositeTypeCompiler(**kwargs)]

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], _problem.goal, domain, _problem.optimization, _problem.opt_func)
        for i in _problem.init:
            if not i.negated:
                p2.init.append(i.copy(new_scope=p2))
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
    def type_visitor(elem, results):
        if isinstance(elem, (conditions.QuantifiedCondition, effects.UniversalEffect)):
            for p in elem.args:
                p.type = CompositeTypeCompiler.replacement_type(p.type, elem.types)
        elif isinstance(elem, effects.ConditionalEffect):
            elem.condition.visit(CompositeTypeCompiler.type_visitor)
    
    def translate_action(self, action, domain=None):
        a2 = action.copy(newdomain=domain)
        for p in a2.args:
            p.type = CompositeTypeCompiler.replacement_type(p.type, domain.types)
        
        visitors.visit(a2.precondition, CompositeTypeCompiler.type_visitor)
        visitors.visit(a2.replan, CompositeTypeCompiler.type_visitor)
        visitors.visit(a2.effect, CompositeTypeCompiler.type_visitor)
        return a2

    def translate_axiom(self, axiom, domain=None):
        a2 = axiom.copy(newdomain=domain)
        for p in a2.args:
            p.type = CompositeTypeCompiler.replacement_type(p.type, domain.types)
        
        a2.condition.visit(CompositeTypeCompiler.type_visitor)
        return a2
        
    def translate_domain(self, _domain):
        dom = domain.Domain(_domain.name, _domain.types.copy(), set(_domain.constants), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        typedict = {}
        dom.requirements = _domain.requirements.copy()
        for func in itertools.chain(dom.predicates, dom.functions):
            for p in func.args:
                p.type = CompositeTypeCompiler.replacement_type(p.type, dom.types)
        
        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom
    

class ObjectFluentNormalizer(Translator):
    def create_param(self, name, type, names):
        param = types.Parameter(name, type )
        i = 1
        while param.name in names:
            param.name = "%s%d" % (name, i)
            i += 1
        return param

    def translate_literal(self, lit, termdict):
        names = set(p.name for p in termdict.itervalues())
        
        def term_visitor(term, results):
            if isinstance(term, FunctionVariableTerm):
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
            if isinstance(parg.type, types.FunctionType):
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

        return lit.__class__(lit.predicate, new_args, negated=lit.negated)

    def translate_condition(self, cond, termdict, scope):
        @visitors.copy
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                pargs = []
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
                        cond = cond.__class__(new_pred, [cond.args[1], cond.args[0]], negated=cond.negated)
                    elif isinstance(cond.args[0], (VariableTerm, ConstantTerm)) and isinstance(cond.args[1], (VariableTerm, ConstantTerm)):
                        return cond
                    if isinstance(cond.args[1], FunctionTerm) and cond.args[0] in termdict and cond.args[1] not in termdict:
                        cond = cond.__class__(new_pred, [cond.args[1], cond.args[0]], negated=cond.negated)

                return self.translate_literal(cond, termdict)
            
        return visitors.visit(cond, visitor)
        
    def translate_effect(self, eff, termdict, scope):
        @visitors.copy
        def visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return self.translate_literal(eff, termdict)

        return visitors.visit(eff, visitor)
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        termdict = {}
        pre = self.translate_condition(action.precondition, termdict, domain)
        replan = self.translate_condition(action.replan, termdict, domain)
        effect = self.translate_effect(action.effect, termdict, domain)
        #print effect.pddl_str()

        args = [types.Parameter(p.name, p.type) for p in action.args]
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(equals, [term, Term(param)]))
                args.append(param)
            # if action.precondition is None:
            #     print pre.pddl_str()
            
        return actions.Action(action.name, args, pre, effect, domain, replan=replan)

    def translate_axiom(self, axiom, domain=None):
        assert domain is not None

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

class ObjectFluentCompiler(Translator):
    def __init__(self, **kwargs):
        self.depends = [MAPLCompiler(**kwargs), ObjectFluentNormalizer(**kwargs)]
        
    def translate_condition(self, cond, scope):
        if cond is None:
            return None, []
        
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == equals:
                    assert isinstance(cond.args[0], FunctionTerm) and isinstance(cond.args[1], (VariableTerm, ConstantTerm))
                    new_pred = scope.predicates.get(cond.args[0].function.name, cond.args[0].args + cond.args[-1:])
                    if not new_pred:
                        print "%s, %s" % (cond.args[0].function.name, ", ".join(str(a.object) for a in cond.args[0].args + cond.args[-1:]))
                        print cond.pddl_str()
                        assert False
                    
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
                if eff.predicate == assign:
                    assert isinstance(eff.args[0], FunctionTerm) and isinstance(eff.args[1], (VariableTerm, ConstantTerm))
                    term = eff.args[0]
                    value = eff.args[1]
                    new_pred = scope.predicates.get(term.function.name, term.args + eff.args[-1:])
                    effs = []
                    if term in previous_values:
                        for val in previous_values[term]:
                            effs.append(effects.SimpleEffect(new_pred, term.args[:] + [val], negated=True))
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
                        effs = [effects.UniversalEffect([param], ceffect, None), effects.SimpleEffect(new_pred, term.args[:] + [value])]
                    return effects.ConjunctiveEffect(effs)

        result = eff.visit(effectsVisitor)
        result.set_scope(scope)
        return result.visit(visitors.flatten)
        
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        a2 = actions.Action(action.name, [types.Parameter(p.name, p.type) for p in action.args], None, None, domain)
        
        a2.precondition, facts = self.translate_condition(action.precondition, a2)
        a2.replan, rfacts = self.translate_condition(action.replan, a2)
        facts += rfacts

        a2.effect = action.effect.copy(new_scope=a2)
        a2.effect = self.translate_effect(action.effect, facts, a2)
            
        return a2
            
    def translate_axiom(self, axiom, domain=None):
        assert domain is not None

        a2 = axioms.Axiom(domain.predicates.get(axiom.predicate.name, axiom.args), [types.Parameter(p.name, p.type) for p in axiom.args], None, domain)
        a2.condition, _ = self.translate_condition(axiom.condition, a2)
        return a2
        
    def translate_domain(self, _domain):
        predicates = []
        for f in _domain.functions:
            predicates.append(Predicate(f.name, [types.Parameter(p.name, p.type) for p in f.args] + [types.Parameter("?value", f.type)]))

        predicates = scope.FunctionTable(predicates)
        for p in _domain.predicates:
            predicates.add(p)

        dom = domain.Domain(_domain.name, _domain.types.copy(), _domain.constants.copy(), predicates, scope.FunctionTable(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.requirements.discard("object-fluents")
        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)
        
        p2.goal,_ = self.translate_condition(_problem.goal, p2)
        
        for i in _problem.init:
            if i.predicate == equal_assign:
                new_pred = domain.predicates.get(i.args[0].function.name, i.args[0].args + i.args[-1:])
                p2.init.append(effects.SimpleEffect(new_pred, i.args[0].args[:] + [i.args[1]], p2))
            else:
                p2.init.append(i.copy(new_scope=p2))
        
        return p2
        
class ModalPredicateCompiler(Translator):
    def __init__(self, **kwargs):
        self.depends = [MAPLCompiler(**kwargs)]
        
    def compile_modal_args(self, args, functions):
        func_arg = None
        funcs = []
        pref = []
        suff = []

        proxy_args = []

        for a in args:
            if isinstance(a.type, types.FunctionType):
                assert func_arg is None, "Only one functional parameter currently possible."
                func_arg = a
                for func in functions:
                    if not func.type.equal_or_subtype_of(a.type.type):
                        continue
                    funcs.append(func)
            else:
                if isinstance(a.type, types.ProxyType):
                    a = types.Parameter(a.name, a.type)
                    proxy_args.append(a)

                if func_arg:
                    suff.append(a)
                else:
                    pref.append(a)

        compiled = []
        for f in funcs:
            for a in proxy_args:
                a.type = f.type
                
            new_args = [types.Parameter(a.name, a.type) for a in pref + f.args + suff]
            compiled.append((f, new_args))

        return func_arg, compiled

    def translate_literal(self, literal, scope):
        if literal.predicate in numeric_ops + assignment_ops:
            return literal.copy_instance()
        
        args = []
        name_elems = [literal.predicate.name]
        has_functionals = False
        for arg, parg in zip(literal.args, literal.predicate.args):
            if isinstance(parg.type, types.FunctionType):
                name_elems.append(arg.function.name)
                args += arg.args
                has_functionals = True
            else:
                args.append(arg)
        if not has_functionals:
            return literal

        pred = scope.predicates.get("-".join(name_elems), args)
        
        result = Literal(pred, args, negated=literal.negated)
        result.__class__ = literal.__class__
        return result.copy_instance()
                
    def translate_action(self, action, domain=None, new_args=None):
        assert domain is not None

        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)

        @visitors.copy
        def eff_visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return self.translate_literal(eff, domain)
            
        if new_args:
            args = new_args
        else:
            args = [types.Parameter(p.name, p.type) for p in action.args]

        a2 = actions.Action(action.name, args, None, None, domain)
        if action.precondition:
            a2.precondition = action.precondition.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
            a2.precondition.set_scope(a2)
        if action.replan:
            a2.replan = action.replan.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
            a2.replan.set_scope(a2)
        a2.effect = visitors.visit(action.effect, eff_visitor)
        
        return a2
            
    def translate_axiom(self, axiom, domain=None, new_args=None, new_pred=None):
        assert domain is not None

        @visitors.replace
        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)
        
        if new_args:
            args = new_args
        else:
            args = [types.Parameter(p.name, p.type) for p in action.args]

        if new_pred:
            pred = new_pred
        else:
            pred = domain.predicates.get(axiom.predicate, args)

        a2 = axioms.Axiom(pred, args, None, domain)
        a2.condition = axiom.condition.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
        a2.condition.set_scope(a2)

        return a2
        
    def translate_domain(self, _domain):
        modal = []
        nonmodal = []
        for pred in _domain.predicates:
            if pred not in numeric_ops + assignment_ops and any(isinstance(a.type, types.FunctionType) for a in pred.args):
                modal.append(pred)
            else:
                nonmodal.append(pred)
        if not modal:
            return _domain.copy()

        funcs = [f for f in _domain.functions if not f.builtin]
        new_preds = []
        for pred in modal:
            func_arg, compiled = self.compile_modal_args(pred.args, funcs)
            for f, args in compiled:
                new_preds.append(Predicate("%s-%s" % (pred.name, f.name), args))

        nonmodal += new_preds
            
        dom = domain.Domain(_domain.name, _domain.types.copy(), _domain.constants.copy(), scope.FunctionTable(nonmodal), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.requirements.discard("modal-predicates")
        
        for ac in _domain.actions:
            func_arg, compiled = self.compile_modal_args(ac.args, funcs)
            if not func_arg:
                dom.actions.append(self.translate_action(ac, dom))
            else:
                for f, args in compiled:
                    ac.instantiate({func_arg : FunctionTerm(f, [Term(a) for a in f.args])})
                    a2 = self.translate_action(ac, dom, args)
                    ac.uninstantiate()
                    a2.name = "%s-%s" % (a2.name, f.name)
                    dom.actions.append(a2)
                    
        for ax in _domain.axioms:
            func_arg, compiled = self.compile_modal_args(ax.args, funcs)
            if not func_arg:
                dom.axioms.append(self.translate_axiom(ax, dom))
            else:
                for f, args in compiled:
                    ax.instantiate({func_arg : FunctionTerm(f, [Term(a) for a in f.args])})
                    new_pred = dom.predicates.get("%s-%s" % (ax.predicate.name, f.name), args)
                    a2 = self.translate_axiom(ax, dom, args, new_pred)
                    a2.copy()
                    ax.uninstantiate()
                    dom.axioms.append(a2)
        dom.stratify_axioms()
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)

        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, p2)
            else:
                return cond.copy(new_parts=results)

        if _problem.goal:
            p2.goal = _problem.goal.visit(cond_visitor).copy(new_scope=p2)
        
        for i in _problem.init:
#            if isinstance(i, effects.ProbabilisticEffect):
#                p2.init.append(effects.ProbabilisticEffect([(p, [self.translate_literal(eff, p2).copy(new_scope=p2) for eff in e]) for p,e in i.effects]))
#            else:
            p2.init.append(self.translate_literal(i, p2).copy(new_scope=p2))
        
        return p2             
    

class MAPLCompiler(Translator):
    def __init__(self, remove_replan=False, **kwargs):
        Translator.__init__(self)
        self.remove_replan = remove_replan
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        a2 = actions.Action(action.name, action.args, None, None, domain)
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
            a2.effect = action.effect.copy(new_scope=a2)
        return a2

    def translate_sensor(self, sensor, domain=None):
        assert domain is not None

        a2 = actions.Action(sensor.name, sensor.args, None, None, domain)
        if sensor.precondition:
            a2.precondition = sensor.precondition.copy(new_scope=a2)
        a2.effect = sensor.knowledge_effect().copy(new_scope=a2)
        return a2
    
    def translate_domain(self, _domain):
        import mapl
        if not isinstance(_domain, mapl.MAPLDomain):
            return _domain
        
        dom = domain.Domain(_domain.name, _domain.types.copy(), _domain.constants.copy(), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.requirements.discard("mapl")
        dom.constants.discard(types.UNKNOWN)

        for func in mapl.modal_predicates:
            dom.predicates.remove(func)
        
        knowledge = Predicate("kval", [Parameter("?a", mapl.t_agent), Parameter("?f", FunctionType(t_object))], builtin=False)
        direct_knowledge = Predicate("kd", [Parameter("?a", mapl.t_agent), Parameter("?f", FunctionType(t_object))], builtin=False)
        p = Parameter("?f", FunctionType(t_object))
        indomain = Predicate("in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=False)
        p = Parameter("?f", FunctionType(t_object))
        i_indomain = Predicate("i_in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=False)

        dom.predicates.add(knowledge)
        dom.predicates.add(direct_knowledge)
        dom.predicates.add(indomain)
        dom.predicates.add(i_indomain)
        
        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.actions += [self.translate_sensor(s, dom) for s in _domain.sensors]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], _problem.goal, domain, _problem.optimization, _problem.opt_func)
        
        for i in _problem.init:
            #determinise probabilistic init conditions
            if isinstance(i, effects.ProbabilisticEffect):
                for p, eff in i.effects:
                    for e in eff.visit(visitors.to_list):
                        if e.predicate in assignmentOps:
                            p2.init.append(effects.SimpleEffect(i_indomain, e.args, scope=p2))
                        else:
                            p2.init.append(e.copy(new_scope=p2))
            elif not i.args or i.args[-1] != builtin.UNKNOWN:
                p2.init.append(i.copy(new_scope=p2))

        return p2
