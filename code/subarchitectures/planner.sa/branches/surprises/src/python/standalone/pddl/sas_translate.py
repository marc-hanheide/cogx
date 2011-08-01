from collections import defaultdict
import itertools

import predicates, conditions, effects, actions, axioms, domain, problem, translators, state
import writer
import visitors
from conditions import *
import mapltypes as types
from sas_task import *

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

@visitors.copy
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

    @visitors.copy
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
    @visitors.copy
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
    @visitors.copy
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

        a2 = actions.Action(action.name, [types.Parameter(p.name, p.type) for p in action.args], None, None, domain)
        
        if action.effect:
            a2.effect = action.effect.copy(new_scope=a2)
        if action.replan:
            a2.replan = action.replan.copy(new_scope=a2)
            
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
        pre = move_disjunction(pre, domain)

        split_axioms = new_axioms
        if isinstance(pre, Disjunction):
            for part in pre.parts:
                ax = axioms.Axiom(a2.predicate, [types.Parameter(p.name, p.type) for p in a2.args], None, domain)
                ax.condition = move_existential(part, domain)
                ax.condition.set_scope(ax)
                split_axioms.append(ax)
        else:
            a2.condition = move_existential(pre, domain)
            a2.condition.set_scope(a2)
            split_axioms.append(a2)

        more_new_axioms = []
        for a in split_axioms:
            if isinstance(a.condition, ExistentialCondition):
                new_args = a.args + a.condition.args
                new_pred = predicates.Predicate("axiom_%d" % new_axiom_count, [types.Parameter(p.name, p.type) for p in new_args])
                globals()["new_axiom_count"] += 1
                domain.predicates.add(new_pred)

                a3 = axioms.Axiom(new_pred, [types.Parameter(p.name, p.type) for p in new_args], None, domain)
                a3.condition = a.condition.condition.copy(new_scope=a3)
                more_new_axioms.append(a3)
                a.condition.condition = LiteralCondition(new_pred, new_args, a.condition)
                
        return split_axioms+more_new_axioms
    
        
    def translate_domain(self, _domain):
        dom = domain.Domain(_domain.name, _domain.types.copy(), set(_domain.constants), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()
        dom.actions += sum([self.translate_action(a, dom) for a in _domain.actions[:]], [])
        dom.axioms += sum([self.translate_axiom(a, dom) for a in _domain.axioms[:]], [])
        dom.stratify_axioms()
        dom.name2action = None
        return dom

ATOM_UNUSED = 0
ATOM_POSITIVE = 1
ATOM_NEGATIVE = 2
    
def get_atom_usage(domain):
    usage = defaultdict(lambda: ATOM_UNUSED)
    @visitors.collect
    def visitor(elem, results):
        if isinstance(elem, predicates.Literal):
            if elem.predicate.builtin:
                function = elem.args[0].function
            else:
                function = elem.predicate
            if elem.negated:
                return (function, ATOM_NEGATIVE)
            else:
                return (function, ATOM_POSITIVE)

    for a in domain.actions:
        for f, use in visitors.visit(a.precondition, visitor, []):
            usage[f] |= use
        for f, use in visitors.visit(a.replan, visitor, []):
            usage[f] |= use

    pred_to_axioms = defaultdict(set)
    for a in domain.axioms:
        pred_to_axioms[a.predicate].add(a)
            
    for level, preds in sorted(domain.stratification.iteritems(), reverse=True):
        changed = True
        while changed:
            changed = False
            for p in preds:
                if usage[p] == ATOM_UNUSED:
                    continue
                for ax in pred_to_axioms[p]:
                    for f, use in ax.condition.visit(visitor):
                        if usage[p] & ATOM_POSITIVE:
                            if not usage[f] & use:
                                usage[f] |= use
                                changed = True
                        if usage[p] & ATOM_NEGATIVE:
                            if use == ATOM_NEGATIVE:
                                use = ATOM_POSITIVE
                            else:
                                use = ATOM_NEGATIVE
                            if not usage[f] & use:
                                usage[f] |= use
                                changed = True
                                
    return usage

def get_nonstatic_functions(domain):
    @visitors.collect
    def visitor(elem, results):
        if isinstance(elem, predicates.Literal):
            if elem.predicate.builtin:
                return elem.args[0].function
            else:
                return elem.predicate
                
    result = set()
    for a in domain.actions:
        result |= set(a.effect.visit(visitor))

    pred_to_axioms = defaultdict(set)
    for a in domain.axioms:
        pred_to_axioms[a.predicate].add(a)
            
    for level, preds in sorted(domain.stratification.iteritems()):
        changed = True
        while changed:
            changed = False
            for p in preds:
                if p in result:
                    continue
                for ax in pred_to_axioms[p]:
                    deps = set(ax.condition.visit(visitor))
                    if deps & result:
                        result.add(p)
                        changed = True
            
    return result

def get_static_functions(domain):
    nonstatic = get_nonstatic_functions(domain)
    result = set()
    for func in itertools.chain(domain.predicates, domain.functions):
        if func not in nonstatic:
            result.add(func)

    return result

def get_sas_vars(svar, value, sas_dict, sas_ranges):
    if svar not in sas_dict:
        var = len(sas_dict)
        sas_dict[svar] = var
        sas_ranges[var] = {}
    else:
        var = sas_dict[svar]
        
    if value not in sas_ranges[var]:
        val = len(sas_ranges[var])
        sas_ranges[var][value] = val
    else:
        val = sas_ranges[var][value]
    return (var, val)

def action_to_sas(action, args, sas_dict, sas_ranges, static):
    @visitors.collect
    def cond_visitor(cond, results):
        if isinstance(cond, predicates.Literal):
            svar, val = state.Fact.from_literal(cond)
            if svar.function not in static:
                return (svar, val)

    @visitors.collect
    def eff_visitor(eff, results):
        if isinstance(eff, predicates.Literal):
            svar, val = state.Fact.from_literal(eff)
            return (svar, val)

    prevail = {}
    replan = {}
    action.instantiate(args)
    for svar, val in visitors.visit(action.precondition, cond_visitor, []):
        var,v = get_sas_vars(svar, val, sas_dict, sas_ranges)
        prevail[var] = v
    for svar, val in visitors.visit(action.replan, cond_visitor, []):
        var,v = get_sas_vars(svar, val, sas_dict, sas_ranges)
        replan[var] = v
        prevail[var] = v

    effects = []
    for svar, val in visitors.visit(action.effect, eff_visitor, []):
        var,v = get_sas_vars(svar, val, sas_dict, sas_ranges)
        if var in prevail:
            effects.append((var, prevail[var], v))
            del prevail[var]
        else:
            effects.append((var, -1, v))

    prevail =  [p for p in prevail.iteritems()]
    replan =  [p for p in replan.iteritems()]
                
    op = SASOperator("(%s %s)" % (action.name, " ".join(a.name for a in args)), prevail, replan, effects, 1)
                
    action.uninstantiate()
    return op

def axiom_to_sas(axiom, args, sas_dict, sas_ranges, static, eval_negative=False):
    def cond_visitor(cond, results):
        if isinstance(cond, predicates.Literal):
            svar, val = state.Fact.from_literal(cond)
            if svar.function in static:
                return []
            return [(svar, val)]
        else:
            return sum(results, [])

    prevail = []
    axiom.instantiate(args)
        
    for svar, val in axiom.condition.visit(cond_visitor):
        prevail.append(get_sas_vars(svar, val, sas_dict, sas_ranges))

    this_svar = state.StateVariable(axiom.predicate, args[:len(axiom.predicate.args)])
    var, post = get_sas_vars(this_svar, types.TRUE, sas_dict, sas_ranges)
    sas_ranges[var][types.FALSE] = 1
    
    if eval_negative:
        rules = []
        post = 1
        for var, val in prevail:
            for v2 in sas_ranges[var].itervalues():
                if v2 != val:
                    rules
        
    ax = SASAxiom(prevail, [var, post])
                
    axiom.uninstantiate()
    return [ax]


def to_sas(problem):
    problem = SASTranslator().translate(problem)
    w = writer.Writer()
    s = w.write_domain(problem.domain)
    print "\n".join(s)
    
    static = get_static_functions(problem.domain)
    axiom_usage = get_atom_usage(problem.domain)

    sas_dict = {}
    sas_ranges = {}

    ops = []
    rules = []

    num_actions = 0
    for a in problem.actions:
        for c in instantiate_action(a, state.State.from_problem(problem), static):
            num_actions += 1
            #print "(%s %s)" % (a.name, " ".join(o.name for o in c))
            ops.append(action_to_sas(a, c, sas_dict, sas_ranges, static))
            
    num_axioms = 0
    axioms_of = defaultdict(list)
    for a in problem.axioms:
        if axiom_usage[a.predicate] == ATOM_UNUSED:
            continue
        if axiom_usage[a.predicate] & ATOM_NEGATIVE:
            axioms_of[a.predicate].append(a)
            
        if axiom_usage[a.predicate] & ATOM_POSITIVE:
            #push existential conditions into the axiom head
            if isinstance(a.condition, ExistentialCondition):
                a2 = a.copy()
                a2.args += a2.condition.args
                a2.add(a2.condition.args)
                a2.free_args = a2.condition.args
                a2.condition = a2.condition.condition.copy(new_scope=a2)
                a = a2
            
            for c in instantiate_axiom(a, state.State.from_problem(problem), static):
                num_axioms += 1
                #print "(%s %s)" % (a.predicate.name, " ".join(o.name for o in c))
                rules.append(axiom_to_sas(a, c, sas_dict, sas_ranges, static))

    for p, neg_axioms in axioms_of.iteritems():
        print p.name
        negated_condition = Disjunction([a.condition for a in neg_axioms]).negate()
        negated_condition = move_disjunction(negated_condition.visit(normalize_visitor), problem.domain)
        a2 = axioms.Axiom(p, [types.Parameter(p.name, p.type) for p in p.args], negated_condition, problem)

        print negated_condition.pddl_str()
        
        

    print "Actions: %d" % num_actions
    print "Axioms: %d" % num_axioms
    

class ProxyStateVariable(state.StateVariable):
    def __init__(self, function, args, modality=None, modal_args=[], variable_term=None):
        if variable_term:
            self.term = variable_term
        else:
            self._function = function
            assert len(function.args) == len(args)
            for a, fa in zip(args, function.args):
                assert a.is_instance_of(fa.type), "%s not of type %s" % (str(a), str(fa))
            self._args = args
            self.term = None

        self.mapping = None
        self.modality = modality
        self._modal_args = modal_args
        if modal_args is None:
            self._modal_args = []
#        self.hash = hash((self.function,self.modality)+ tuple(self.args)+tuple(self.modal_args))

    def __get_function(self):
        if self.term:
            if not self.mapping:
                return None
            return self.mapping[self.term.object].function
        return self._function

    def __get_args(self):
        if self.term:
            if not self.mapping:
                return []
            return [a.object for a in self.mapping[self.term.object].args]
        if not self.mapping:
            return self._args
        return [self.mapping.get(a,a) for a in self._args]

    def __get_modal_args(self):
        if not self.mapping:
            return self._modal_args
        return [self.mapping.get(a, a) for a in self._modal_args]
    
    function = property(__get_function)
    args = property(__get_args)
    modal_args = property(__get_modal_args)
    
    def get_type(self):
        if self.modality:
            return self.modality.type
        else:
            return self.function.type

    def get_predicate(self):
        if self.modality:
            return self.modality
        if isinstance(self.function, predicates.Predicate):
            return self.function
        return None

    def instantiate(self, mapping):
        self.mapping = mapping
        
    def uninstantiate(self):
        self.mapping = None
    
        
    def __str__(self):
        s = "%s(%s)" % (self.function.name, " ".join(a.name for a in self.args))
        if self.modality:
            s = "%s(%s, %s)" % (self.modality.name, s, " ".join(a.name for a in self.modal_args))
        return s

    def __eq__(self, other):
        return isinstance(other, state.StateVariable) and hash(self) == hash(other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.function,self.modality)+ tuple(self.args)+tuple(self.modal_args))
    
    @staticmethod
    def from_literal(literal, state=None):
        fterm = None
        modal_args = []
        if literal.predicate in predicates.assignmentOps + [predicates.equals]:
            fterm = literal.args[0]
            modal_args = None
        else:
            for arg, parg in zip(literal.args, literal.predicate.args):
                if isinstance(parg.type, types.FunctionType):
                    if fterm is None:
                        fterm = arg
                    else:
                        assert False, "A modal svar can only contain one function"
                else:
                    modal_args.append(arg)

            if not fterm:
                fterm = literal
                modal_args = None

        modality = None
        if modal_args is not None:
            modal_args = [a.object for a in modal_args]
            modality = literal.predicate

        if fterm == literal:
            return ProxyStateVariable(fterm.predicate, [a.object for a in fterm.args], modality, modal_args)
        elif fterm.__class__ == predicates.FunctionVariableTerm:
            return ProxyStateVariable(None, [], modality, modal_args, variable_term=fterm)
        else:
            return ProxyStateVariable(fterm.function, [a.object for a in fterm.args], modality, modal_args)

def instantiate_action(action, st, check_functions):
    cond = Conjunction([])
    if action.precondition:
        cond.parts.append(action.precondition)
    if action.replan:
        cond.parts.append(action.replan)
    return instantiate(cond, action.args, st, check_functions)

def instantiate_axiom(axiom, st, static_functions):
    if axiom.predicate in static_functions:
        return []
    
    return instantiate(axiom.condition, axiom.args, st, static_functions)

def instantiate(condition, args, st, check_functions):
    def getConditionVisitor(cond, results):
        if isinstance(cond, LiteralCondition):
            #print cond.pddl_str()
            if cond.predicate in predicates.numericComparators + [predicates.equals]:
                if cond.args[0].function in check_functions:
                    assert not cond.negated
                    return [(ProxyStateVariable.from_literal(cond), cond.args[1].object)]

            elif cond.predicate in check_functions:
                if cond.negated:
                    return [(ProxyStateVariable.from_literal(cond), types.FALSE)]
                return [(ProxyStateVariable.from_literal(cond), types.TRUE)]
            return []

        elif isinstance(cond, Truth):
            return []
        elif isinstance(cond, Conjunction):
            return sum(results, [])
        assert False

    def check_recursive(mapping, uninst, open_conds):
        p = uninst[0]
        #print "now:", p
        for v in st.problem.get_all_objects(p.type):
            m = mapping.copy()
            m[p] = v
            fulfilled = True
            new_conds = []
            for svar, value in open_conds:
                svar.instantiate(m)
                #print svar, svar in st
                #print map(str, svar.args)
                if all(a.__class__ != types.Parameter for a in itertools.chain(svar.args, svar.modal_args)):
                    exst = st.get_extended_state([svar])
                    if svar not in exst:
                        #sv2 = state.StateVariable(svar.function, svar.args)
                        #print sv2, sv2 in st
                        fulfilled = False
                        break
                    real_val = exst[svar]
                    if value in m and real_val != m[value]:
                        #print "mismatch:", svar, m[value], " != ", real_val
                        fulfilled = False
                        break
                    elif value not in m:
                        #print "set:", value, " to ", real_val
                        m[value] = real_val
                    #print "consistent:", real_val
                else:
                    new_conds.append((svar, value))
            if not fulfilled:
                continue
            uninst = [u for u in uninst if u not in m]
            if not uninst:
                yield m
            else:
                for mp in check_recursive(m, uninst, new_conds):
                    yield mp

    for m in check_recursive({}, args, condition.visit(getConditionVisitor)):
        #print [a.name for a in action.args]
        #print [a.name for a in m.iterkeys()]
        yield [m[a] for a in args]
    
#    for c in combinations:
#        action.instantiate(c)
#        if action.precondition.visit(checkConditionVisitor):
#            yield c
#        action.uninstantiate()
