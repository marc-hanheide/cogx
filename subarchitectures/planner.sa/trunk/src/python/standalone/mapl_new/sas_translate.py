from collections import defaultdict
import time

import predicates, conditions, effects, actions, axioms, domain, problem, translators, state
from conditions import *
import mapltypes as types

new_axiom_count=0

def product(*iterables):
    if not iterables:
        yield tuple()
        return
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)

def copy_visitor(f):
    def decorated_visitor(elem, results):
        result = f(elem, results)
        if result:
            return result
        return elem.copy(new_parts = results)
    return decorated_visitor

@copy_visitor
def normalize_visitor(cond, results):
    if isinstance(cond, Conjunction):
        new_parts = []
        for p in cond.parts:
            if isinstance(p, Conjunction):
                new_parts += p.parts
            else:
                new_parts.append(p)
        return Conjunction(new_parts)
    elif isinstance(cond, Disjunction):
        new_parts = []
        for p in cond.parts:
            if isinstance(p, Disjunction):
                new_parts += p.parts
            else:
                new_parts.append(p)
        return Disjunction(new_parts)
    elif isinstance(cond, QuantifiedCondition):
        if cond.condition.__class__ == cond.__class__:
            return cond.__class__(cond.args + cond.condition.args, cond.condition.condition, cond.parent)
    
def eliminate_universal(cond, domain):
    new_axioms = []

    @copy_visitor
    def visitor(cond, results):
        if isinstance(cond, UniversalCondition):
            free = list(cond.free())
            new_pred = predicates.Predicate("axiom_%d" % new_axiom_count, [types.Parameter(p.name, p.type) for p in free])
            globals()["new_axiom_count"] += 1
            domain.predicates.add(new_pred)
            
            new_args = [types.Parameter(p.name, p.type) for p in free]
            axiom = axioms.Axiom(new_pred, new_args, None, domain)
            axiom.condition = ExistentialCondition([types.Parameter(p.name, p.type) for p in cond.args], results[0].negate(), axiom)
            new_axioms.append(axiom)
            return LiteralCondition(new_pred, free, negated=True)
    return cond.visit(visitor).visit(normalize_visitor), new_axioms

def move_disjunction(cond, domain):
    @copy_visitor
    def visitor(cond, results):
        if isinstance(cond, ExistentialCondition):
            if isinstance(cond.condition, Disjunction):
                return Disjunction([cond.copy(new_parts=[p]) for p in cond.condition.parts])
        elif isinstance(cond, Conjunction):
            dis_parts = [p.parts for p in results if isinstance(p, Disjunction)]
            if not dis_parts:
                return cond
            nondis = [p for p in results if not isinstance(p, Disjunction)]
            return Disjunction(Conjunction([d] + nondis) for d in product(*dis_parts))
    return cond.visit(visitor).visit(normalize_visitor)

def move_existential(cond, domain):
    @copy_visitor
    def visitor(cond, results):
        if isinstance(cond, Conjunction):
            ex = [p for p in results if isinstance(p, ExistentialCondition)]
            if not ex:
                return cond
            scope = ex[0].parent
            nonex = [p for p in results if not isinstance(p, ExistentialCondition)]
            args = sum([p.args for p in ex], [])
            return ExistentialCondition(args, Conjunction([c.condition for c in ex] + nonex), scope)
    return cond.visit(visitor).visit(normalize_visitor)


class SASTranslator(translators.Translator):
    """ Translate a PDDL domain into a format suitable for converting into an SAS+ task"""
    def __init__(self):
        self.depends = [translators.ModalPredicateCompiler(), translators.ObjectFluentNormalizer()]

    def translate_action(self, action, domain=None):
        assert domain is not None

        a2 = actions.Action(action.name, [types.Parameter(p.name, p.type) for p in action.args], None, [], domain)
        for e in action.effects:
            a2.effects.append(e.copy(new_scope=a2))

        split_actions = []
        if action.precondition:
            pre, axioms = eliminate_universal(action.precondition, domain)
            domain.axioms += axioms
            pre = move_disjunction(pre, domain)
            
            if isinstance(pre, Disjunction):
                for part in pre.parts:
                    ac = a2.copy()
                    ac.precondition = move_existential(part, domain)
                    ac.precondition.set_scope(ac)
                    split_actions.append(ax)
            else:
                a2.precondition = move_existential(pre, domain)
                a2.precondition.set_scope(a2)
                split_actions = [a2]
        else:
            split_actions = [a2]

        for a in split_actions:
            if isinstance(a.precondition, ExistentialCondition):
                a.args += a.precondition.args
                a.precondition = a.precondition.condition
            
        return split_actions

    def translate_axiom(self, axiom, domain=None):
        assert domain is not None

        a2 = axioms.Axiom(axiom.predicate, [types.Parameter(p.name, p.type) for p in axiom.args], None, domain)
        
        pre, new_axioms = eliminate_universal(axiom.condition, domain)
        domain.axioms += new_axioms
        pre = move_disjunction(pre, domain)

        split_axioms = []
        if isinstance(pre, Disjunction):
            for part in pre.parts:
                ax = axioms.Axiom(a2.predicate, [types.Parameter(p.name, p.type) for p in a2.args], None, domain)
                ax.condition = move_existential(part, domain)
                ax.condition.set_scope(ax)
                split_axioms.append(ax)
        else:
            a2.condition = move_existential(pre, domain)
            a2.condition.set_scope(a2)
            split_axioms = [a2]

        return split_axioms
    
        
    def translate_domain(self, _domain):
        dom = domain.Domain(_domain.name, _domain.types.copy(), set(_domain.constants), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.actions += sum([self.translate_action(a, dom) for a in _domain.actions[:]], [])
        dom.axioms += sum([self.translate_axiom(a, dom) for a in _domain.axioms[:]], [])
        dom.stratifyAxioms()
        dom.name2action = None
        return dom


def instantiate(action, st, check_functions):
    import mapl
    action = mapl.MAPLObjectFluentNormalizer().translate(action, domain=st.problem.domain)
    
    combinations = product(*map(lambda arg: list(st.problem.getAll(arg.type)), action.args))
    def getConditionVisitor(cond, results):
        if isinstance(cond, LiteralCondition):
            print cond.pddl_str()
            if cond.predicate in predicates.numericComparators + [predicates.equals]:
                if cond.args[0].function in check_functions:
                    assert not cond.negated
                    return [((cond.args[0].function, [a.object for a in cond.args[0].args]), cond.args[1].object)]

            elif cond.predicate in check_functions:
                return [((cond.predicate, [a.object for a in cond.args]), types.TRUE)]
            return []

        elif isinstance(cond, Truth):
            return []
        elif isinstance(cond, Conjunction):
            return sum(results, [])
        assert False

    def check_recursive(mapping, uninst, open_conds):
        p = uninst[0]
        #print "now:", p
        for v in st.problem.getAll(p.type):
            m = mapping.copy()
            m[p] = v
            fulfilled = True
            new_conds = []
            for (func, args), value in open_conds:
                #print func.name, [a.name for a in args], value
                if all(a in m for a in args):
                    real_val = st[state.StateVariable(func, [m[a] for a in args])]
                    if value in m and real_val != m[value]:
                        #print "mismatch:", func.name, [a.name for a in args], m[value], " != ", real_val
                        fulfilled = False
                        break
                    elif value not in m:
                        #print "set:", value, " to ", real_val
                        m[value] = real_val
                else:
                    new_conds.append(((func, args), value))
            if not fulfilled:
                continue
            uninst = [u for u in uninst if u not in m]
            if not uninst:
                yield m
            else:
                for mp in check_recursive(m, uninst, new_conds):
                    yield mp

    for m in check_recursive({}, action.args, action.precondition.visit(getConditionVisitor)):
        yield [m[a] for a in action.args]
    
#    for c in combinations:
#        action.instantiate(c)
#        if action.precondition.visit(checkConditionVisitor):
#            yield c
#        action.uninstantiate()
