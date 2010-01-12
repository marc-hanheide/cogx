from collections import defaultdict
import time

import predicates, conditions, effects, actions, axioms, domain, problem
import mapltypes as types
from predicates import *

class Translator(object):
    def __init__(self):
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
        dom.stratifyAxioms()
        dom.name2action = None
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        return problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
    
class ADLCompiler(Translator):
    def __init__(self):
        self.depends = [ModalPredicateCompiler(), ObjectFluentCompiler()]

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], _problem.goal, domain, _problem.optimization, _problem.opt_func)
        for i in _problem.init:
            if not i.negated:
                p2.init.append(i.copy(new_scope=p2))
        return p2
        
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
        if lit.predicate in numericComparators + [equals]:
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
        names = set(p.name for p in termdict.itervalues())
                    
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                pargs = []
                if cond.predicate in numericComparators + [equals]:
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
            else:
                return cond.copy(new_parts=results)
            
        result = cond.visit(visitor)
        return result
        
    def translate_effect(self, eff, termdict, scope):
        names = set(p.name for p in termdict.itervalues())
                
        def visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return [self.translate_literal(eff, termdict)]
            elif isinstance(eff, list):
                return sum(results, [])
            else:
                #TODO: universal/conditional effects
                effs = sum(results, [])
                return [eff.copy(new_parts=effs)]
        result = eff.visit(visitor)
        return result[0]
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        termdict = {}
        pre = None
        replan = None
        effects = []
        if action.precondition:
            pre = self.translate_condition(action.precondition, termdict, domain)
        if action.replan:
            replan = self.translate_condition(action.replan, termdict, domain)
        for e in action.effects:
            effects.append(self.translate_effect(e, termdict, domain))

        args = [types.Parameter(p.name, p.type) for p in action.args]
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(equals, [term, Term(param)]))
                args.append(param)

        return actions.Action(action.name, args, pre, effects, domain, replan=replan)

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
    def __init__(self):
        self.depends = [MAPLCompiler(), ObjectFluentNormalizer()]
        
    def translate_condition(self, cond, scope):
        def visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == predicates.equals:
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
        previous_values = defaultdict(set)
        for term, value in read_facts:
            previous_values[term].add(value)
                
        def effectsVisitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate == predicates.assign:
                    assert isinstance(eff.args[0], FunctionTerm) and isinstance(eff.args[1], (VariableTerm, ConstantTerm))
                    term = eff.args[0]
                    new_pred = scope.predicates.get(term.function.name, term.args + eff.args[-1:])
                    if term in previous_values:
                        effs = []
                        for val in previous_values[term]:
                            effs.append(effects.SimpleEffect(new_pred, term.args[:] + [val], negated=True))
                        effs.append(effects.SimpleEffect(new_pred, term.args[:] + [eff.args[1]]))
                        return effs
                    else:
                        param = types.Parameter("?oldval", term.function.type)
                        condition = conditions.LiteralCondition(predicates.equals, [eff.args[1], predicates.Term(param)], negated=True)
                        negeffect = effects.SimpleEffect(new_pred, term.args[:] + [predicates.Term(param)], negated=True)
                        ceffect = effects.ConditionalEffect(condition, [negeffect])
                        return [effects.UniversalEffect([param], [ceffect], None), effects.SimpleEffect(new_pred, term.args[:] + [eff.args[1]])]
                return [eff]
            elif isinstance(eff, list):
                return sum(results, [])
            else:
                effs = sum(results, [])
                return [cond.copy(new_parts=effs)]
        return eff.visit(effectsVisitor)
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        a2 = actions.Action(action.name, [types.Parameter(p.name, p.type) for p in action.args], None, [], domain)
        
        pre = None
        facts = []
        if action.precondition:
            a2.precondition, facts = self.translate_condition(action.precondition, a2)
        if action.replan:
            a2.replan, rfacts = self.translate_condition(action.replan, a2)
            facts += rfacts
            
        for e in action.effects:
            a2.effects += self.translate_effect(e, facts, a2)
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
        dom.stratifyAxioms()
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)
        
        p2.goal,_ = self.translate_condition(_problem.goal, p2)
        
        for i in _problem.init:
            if i.predicate == equalAssign:
                new_pred = domain.predicates.get(i.args[0].function.name, i.args[0].args + i.args[-1:])
                p2.init.append(effects.SimpleEffect(new_pred, i.args[0].args[:] + [i.args[1]], p2))
            else:
                p2.init.append(i.copy(new_scope=p2))
        
        return p2             
        
class ModalPredicateCompiler(Translator):
    def __init__(self):
        self.depends = [MAPLCompiler()]
        
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
                    if not func.type.equalOrSubtypeOf(a.type.type):
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
        if literal.predicate in numericOps + assignmentOps:
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

        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)
            elif isinstance(cond, conditions.JunctionCondition):
                cond.parts = results
            elif isinstance(cond, conditions.QuantifiedCondition):
                cond.condition = results[0]
            return cond
            
        def eff_visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                return [self.translate_literal(eff, domain)]
            elif isinstance(eff, list):
                return sum(results, [])
            else:
                return [eff.copy(new_parts=sum(results, []))]
            
        if new_args:
            args = new_args
        else:
            args = [types.Parameter(p.name, p.type) for p in action.args]

        a2 = actions.Action(action.name, args, None, [], domain)
        if action.precondition:
            a2.precondition = action.precondition.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
            a2.precondition.set_scope(a2)
        if action.replan:
            a2.replan = action.replan.copy(copy_instance=True, new_scope=a2).visit(cond_visitor)
            a2.replan.set_scope(a2)
        for e in action.effects:
            a2.effects.append(e.visit(eff_visitor)[0].copy(new_scope=a2))
        return a2
            
    def translate_axiom(self, axiom, domain=None, new_args=None, new_pred=None):
        assert domain is not None

        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, domain)
            elif isinstance(cond, conditions.JunctionCondition):
                cond.parts = results
            elif isinstance(cond, conditions.QuantifiedCondition):
                cond.condition = results[0]
            return cond
        
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
            if pred not in numericOps + assignmentOps and any(isinstance(a.type, types.FunctionType) for a in pred.args):
                modal.append(pred)
            else:
                nonmodal.append
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
        dom.stratifyAxioms()
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], None, domain, _problem.optimization, _problem.opt_func)

        def cond_visitor(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                return self.translate_literal(cond, scope)
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
    def __init__(self, remove_replan=False):
        Translator.__init__(self)
        self.remove_replan = remove_replan
        
    def translate_action(self, action, domain=None):
        assert domain is not None

        a2 = actions.Action(action.name, action.args, None, [], domain)
        if action.precondition:
            a2.precondition = action.precondition.copy(new_scope=a2)
        if action.replan:
            if self.remove_replan:
                parts = [a2.precondition]
                parts.append(action.replan.copy(new_scope=a2))
                a2.precondition = conditions.Conjunction(parts)
            else:
                a2.replan = action.replan.copy(new_scope=a2)
                
        for e in action.effects:
            a2.effects.append(e.copy(new_scope=a2))
        return a2

    def translate_sensor(self, sensor, domain=None):
        assert domain is not None

        a2 = actions.Action(sensor.name, sensor.args, None, [], domain)
        if sensor.precondition:
            a2.precondition = sensor.precondition.copy(new_scope=a2)
        a2.effects = [sensor.knowledge_effect().copy(new_scope=a2)]
        return a2
    
    def translate_domain(self, _domain):
        import mapl
        if not isinstance(_domain, mapl.MAPLDomain):
            return _domain
        
        dom = domain.Domain(_domain.name, _domain.types.copy(), _domain.constants.copy(), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.requirements.discard("mapl")
        dom.constants.discard(types.UNKNOWN)

        for func in predicates.mapl_modal_predicates:
            dom.predicates.remove(func)
        
        knowledge = Predicate("kval", [Parameter("?a", agentType), Parameter("?f", FunctionType(objectType))], builtin=False)
        direct_knowledge = Predicate("kd", [Parameter("?a", agentType), Parameter("?f", FunctionType(objectType))], builtin=False)
        p = Parameter("?f", FunctionType(objectType))
        indomain = Predicate("in-domain", [p, Parameter("?v", ProxyType(p)), ], builtin=False)
        p = Parameter("?f", FunctionType(objectType))
        i_indomain = Predicate("i_in-domain", [p, Parameter("?v", ProxyType(p)), ], builtin=False)

        dom.predicates.add(knowledge)
        dom.predicates.add(direct_knowledge)
        dom.predicates.add(indomain)
        dom.predicates.add(i_indomain)
        
        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.actions += [self.translate_sensor(s, dom) for s in _domain.sensors]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratifyAxioms()
        dom.name2action = None
        return dom
