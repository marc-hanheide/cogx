#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools
from itertools import imap, ifilter
import time
from collections import defaultdict

import mapl_new as mapl
from mapl_new.predicates import *
from mapl_new import problem, conditions, effects, types

def product(*iterables):
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)
                       

def instantiate_args(args, state=None):
    result = []
    for arg in args:
        if isinstance(arg, VariableTerm):
            assert arg.object.isInstantiated(), "%s is not instantiated" % arg.object.name
            arg = arg.copyInstance()

        if isinstance(arg, ConstantTerm):
            result.append(arg.object)
        elif state is not None:
            svar = StateVariable.fromTerm(arg, state)
            if not svar in state:
                raise Exception("couldn't create state variable, %s is not in state." % str(arg))
            value = state[svar]
            result.append(value)
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    return result

def get_svars_from_term(term, state=None):
    svars = set()
    const_args = []
    for arg in term.args:
        if isinstance(arg, VariableTerm):
            assert arg.object.isInstantiated()
            arg = arg.copyInstance()

        if isinstance(arg, ConstantTerm):
            const_args.append(arg.object)
        elif state is not None:
            svar, total = get_svars_from_term(arg, state)
            if not svar in state:
                raise Exception("couldn't create state variable, %s is not in state." % str(arg))
            value = state[svar]
            const_args.append(value)
            svars |= total
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    svar = StateVariable(term.function, const_args)
    svars.add(svar)
    return svar, svars

def get_nonmodal_svars(literal, state):
    svars = set()
    for arg, parg in zip(literal.args, literal.predicate.args):
        if isinstance(arg, FunctionTerm) and not isinstance(parg.type, FunctionType):
            _, all_vars = get_svars_from_term(arg, state)
            svars|= all_vars
        elif isinstance(arg, FunctionTerm):
            for arg2 in arg.args:
                if isinstance(arg2, FunctionTerm):
                    _, all_vars = get_svars_from_term(arg2, state)
                    svars|= all_vars
            
    return svars

class StateVariable(object):
    def __init__(self, function, args, modality=None, modal_args=[]):
        self.function = function
        assert len(function.args) == len(args)
        for a, fa in zip(args, function.args):
            assert a.isInstanceOf(fa.type), "%s not of type %s" % (str(a), str(fa))
        self.args = args

        self.modality = modality
        self.modal_args = modal_args
        self.hash = hash((self.function,self.modality)+ tuple(self.args)+tuple(self.modal_args))

    def get_type(self):
        if self.modality:
            return self.modality.type
        else:
            return self.function.type

    def getPredicate(self):
        if self.modality:
            return self.modality
        if isinstance(self.function, Predicate):
            return self.function
        return None

    def asModality(self, modality, modal_args):
        return StateVariable(self.function, self.args, modality, modal_args)

    def nonmodal(self):
        return StateVariable(self.function, self.args, None, [])
    
    def getArgs(self):
        if not self.modality:
            return self.args
        fterm = FunctionTerm(self.function, [ConstantTerm(a) for a in self.args])
        args = []
        it = iter(self.modal_args)
        for parg in self.modality.args:
            if isinstance(parg.type, FunctionType):
                args.append(fterm)
            else:
                args.append(it.next())
        return args

    def asLiteral(self):
        assert isinstance(self.function, Predicate) or self.modality is not None
        
        if not self.modality:
            return Literal(self.function, [ConstantTerm(a) for a in self.args])
        fterm = FunctionTerm(self.function, [ConstantTerm(a) for a in self.args])
        args = []
        it = iter(self.modal_args)
        for parg in self.modality.args:
            if isinstance(parg.type, FunctionType):
                args.append(fterm)
            else:
                args.append(ConstantTerm(it.next()))
        return Literal(self.modality, args)
        
    def __str__(self):
        s = "%s(%s)" % (self.function.name, " ".join(map(lambda a: a.name, self.args)))
        if self.modality:
            s = "%s(%s, %s)" % (self.modality.name, s, " ".join(map(lambda a: a.name, self.modal_args)))
        return s

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.hash == other.hash
        #return self.__class__ == other.__class__ and self.function == other.function and all(map(lambda a,b: a==b, self.args, other.args)) and self.modality == other.modality and all(map(lambda a,b: a==b, self.modal_args, other.modal_args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.hash

    @staticmethod
    def fromTerm(term, state=None):
        assert isinstance(term, FunctionTerm)
        args = instantiate_args(term.args, state)
        return StateVariable(term.function, args)

    @staticmethod
    def svarsInTerm(term, state=None):
        assert isinstance(term, FunctionTerm)
        svar, vars = get_svars_from_term(term, state)
        vars.add(svar)
        return svar, vars
    
    @staticmethod
    def fromLiteral(literal, state=None):
        function = None
        litargs = None
        modal_args = []
        if literal.predicate in assignmentOps + [equals]:
            function = literal.args[0].function
            litargs = literal.args[0].args
        else:
            for arg, parg in zip(literal.args, literal.predicate.args):
                if isinstance(parg.type, FunctionType):
                    if function is None:
                        function = arg.function
                        litargs = arg.args
                    else:
                        assert False, "A modal svar can only contain one function"
                else:
                    modal_args.append(arg)

            if not function:
                function = literal.predicate
                litargs = literal.args
                modal_args = None

        args = instantiate_args(litargs, state)
        if modal_args:
            modal_args = instantiate_args(modal_args, state)
            return StateVariable(function, args, literal.predicate, modal_args)
            
        return StateVariable(function, args)
    

class Fact(tuple):
    def __new__(_class, svar, value):
        #assert value.isInstanceOf(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        return tuple.__new__(_class, (svar, value))
    
    svar = property(lambda self: self[0])
    value = property(lambda self: self[1])
    
    def asLiteral(self, useEqual=False):
        if isinstance(self.svar.function, Predicate) or self.svar.modality is not None:
            lit = self.svar.asLiteral()
            if self.value == types.TRUE:
                return lit
            elif self.value == types.FALSE:
                return lit.negate()
            
            assert False

        if useEqual:
            op = equalAssign
        else:
            op = assign
            
        return Literal(op, [Term(self.svar.function, [Term(a) for a in self.svar.args]), Term(self.value)])
    
    def __str__(self):
        return "%s = %s" % (str(self.svar), str(self.value))

    @staticmethod
    def fromLiteral(literal, state=None):
        value = None
        if literal.predicate in assignmentOps + [equals]:
            value = instantiate_args(literal.args[-1:], state)[0]
        else:
            if literal.negated:
                value = types.FALSE
            else:
                value = types.TRUE

        return Fact(StateVariable.fromLiteral(literal, state), value)
        
    @staticmethod
    def fromCondition(condition, state=None):
        def factVisitor(cond, facts):
            if isinstance(cond, conditions.LiteralCondition):
                return [Fact.fromLiteral(cond, state)]
            elif isinstance(cond, conditions.Conjunction):
                return sum(facts, [])
            elif isinstance(cond, conditions.Truth):
                return []
            else:
                raise Exception("can't get facts for %s" % cond.__class__)
        return condition.visit(factVisitor)

    @staticmethod
    def fromEffect(effect, state=None):
        if not isinstance(effect, effects.SimpleEffect):
            raise  Exception("can't get facts for quantified or conditional effects")
        return Fact.fromLiteral(effect, state)
    
    @staticmethod
    def fromTuple(tup):
        return Fact(tup[0], tup[1])
    
class State(defaultdict):
    def __init__(self, facts=[], prob=None):
        defaultdict.__init__(self, lambda: mapl.types.UNKNOWN)
        assert prob is None or isinstance(prob, problem.Problem)
        self.problem = prob
        for f in facts:
            self.set(f)

    def copy(self):
        s = State([], self.problem)
        for svar,val in self.iteritems():
            s[svar] = val
        return s

    def iterfacts(self):
        return imap(lambda tup: Fact.fromTuple(tup), self.iteritems())

    def set(self, fact):
        self[fact.svar] = fact.value
    
    def __setitem__(self, svar, value):
        assert isinstance(svar, StateVariable)
        assert value.isInstanceOf(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        dict.__setitem__(self, svar, value)

    def __contains__(self, key):
        if isinstance(key, Fact):
            return dict.__contains__(self, key.svar) and self[key.svar] == key.value
        return dict.__contains__(self, key)

    @staticmethod
    def fromProblem(problem):
        s = State([], problem)
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                s.applyEffect(i)
            else:
                s.set(Fact.fromLiteral(i))
        return s
    
    def factsFromCondition(self, cond):
        return Fact.fromCondition(cond, self)

    def isExecutable(self, action):
        extstate = self.getExtendedState(self.getRelevantVars(action.precondition))
        return extstate.isSatisfied(action.precondition)
            
    def isSatisfied(self, cond, relevantVars=None, universal=None):
        def instantianteAndCheck(cond, params):
            cond.instantiate(dict(zip(cond.args, params)))
            result = checkConditionVisitor(cond.condition)
            cond.uninstantiate()
            return result

        def allFacts(iterable):
            newfacts = []
            newuniversal = []
            for result, facts, universal in iterable:
                if not result:
                    return False, [], []
                newfacts += facts
                newuniversal += universal
            return True, newfacts, newuniversal

        def anyFacts(iterable):
            for result, facts, universal in iterable:
                if result:
                    return True, facts, universal
            return False, [], []
        
        def checkConditionVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                fact = Fact.fromLiteral(cond, self)
                result = fact in self
                if cond.negated:
                    result = not result
                    
                relVars = []
                if relevantVars is not None and result:
                    relVars.append(fact.svar)
                    relVars += list(get_nonmodal_svars(cond, self))
                            
                return result, relVars, []

            elif isinstance(cond, conditions.Truth):
                return True, [], []
            elif isinstance(cond, conditions.Falsity):
                return False, [], []
            elif isinstance(cond, conditions.Conjunction):
                if not cond.parts:
                    return True, [], []

                return allFacts(imap(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.Disjunction):
                return anyFacts(imap(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                combinations = product(*map(lambda a: self.problem.getAll(a.type), cond.args))
                if isinstance(cond, conditions.UniversalCondition):
                    result, vars, universal = allFacts(imap(lambda c: instantianteAndCheck(cond, c), combinations))
                    universal += cond.args
                    return result, vars, universal
                elif isinstance(cond, conditions.ExistentialCondition):
                    return anyFacts(imap(lambda c: instantianteAndCheck(cond, c), combinations))
            elif isinstance(cond, conditions.TimedCondition):
                #ignore times specifiers for now
                return checkConditionVisitor(cond.condition)
            elif cond is None:
                return True, [], []
            assert False

        result, svars, univ = checkConditionVisitor(cond)
        if relevantVars is not None:
            relevantVars += svars
        if universal is not None:
            universal += univ
        return result

    def getRelevantVars(self, cond, restrict_to=None):
        def restrictionVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                return cond.predicate in restrict_to
            elif isinstance(cond, conditions.JunctionCondition):
                return any(imap(restrictionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                return restrictionVisitor(cond.condition)
            else:
                return False

        def dependenciesVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                fact = Fact.fromLiteral(cond, self)
                result = fact in self
                if cond.negated:
                    result = not result
                    
                relVars = [fact.svar]
                for arg in cond.args:
                    if isinstance(arg, FunctionTerm):
                        _, vars = StateVariable.svarsInTerm(arg, self)
                        relVars += vars
                            
                return relVars

            elif isinstance(cond, conditions.JunctionCondition):
                return sum(imap(dependenciesVisitor, cond.parts), [])
            elif isinstance(cond, conditions.QuantifiedCondition):
                if restrict_to:
                    if not restrictionVisitor(cond):
                        return []
                    
                combinations = product(*map(lambda a: self.problem.getAll(a.type), cond.args))
                result = []
                for c in combinations:
                    cond.instantiate(dict(zip(cond.args, c)))
                    result += dependenciesVisitor(cond.condition)
                    cond.uninstantiate()
                return result
            else:
                return []
            
        return set(dependenciesVisitor(cond))

    def getEffectFacts(self, effect):
        result = set()
        if isinstance(effect, effects.UniversalEffect):
            combinations = product(*map(lambda a: self.problem.getAll(a.type), effect.args))
            for params in combinations:
                effect.instantiate(zip(effect.args, params))
                for eff in effect.effects:
                    result |= self.getEffectFacts(eff)
                effect.uninstantiate()
                
        elif isinstance(effect, effects.ConditionalEffect):
            extstate = self.getExtendedState(self.getRelevantVars(effect.condition))
            if extstate.isSatisfied(effect.condition):
                for eff in effect.effects:
                    result |= self.getEffectFacts(eff)

        elif isinstance(effect, effects.ProbabilisticEffect):
            for eff in effect.getRandomEffect():
                result |= self.getEffectFacts(eff)

        elif isinstance(effect, list):
            for eff in effect:
                result |= self.getEffectFacts(eff)
            
        else:
            fact = Fact.fromEffect(effect)
            result.add(fact)
        return result
    
    def applyEffect(self, effect):
        for fact in self.getEffectFacts(effect):
            self.set(fact)
            
    def getExtendedState(self, svars=None, getReasons=False):
        """Evaluate all axioms neccessary to instantiate the variables in 'svars'."""
        t0 = time.time()
        pred_to_axioms = defaultdict(set)
        for a in self.problem.axioms:
            pred_to_axioms[a.predicate].add(a)
        derived = pred_to_axioms.keys()

        def getDependencies(svar, derived):
            open = set([svar])
            closed = set()
            while open:
                sv = open.pop()
                closed.add(sv)
                for ax in self.problem.axioms:
                    if ax.predicate == sv.getPredicate():
                        ax.instantiate(sv.getArgs())
                        for dep in self.getRelevantVars(ax.condition, derived):
                            if dep.getPredicate() in derived and dep not in closed:
                                open.add(dep)
                        ax.uninstantiate()
            return closed
                
        relevant = set()
        if svars is not None:
            for s in svars:
                if s.getPredicate() in derived:
                    relevant |= getDependencies(s, derived)
            if not relevant:
                if getReasons:
                    return self, {}, {}
                return self


        if getReasons:
            reasons = defaultdict(set)
            universalReasons = defaultdict(set)

        #print "finding releveant:", time.time()-t0
        t0 = time.time()
        for level, preds in sorted(self.problem.stratification.iteritems()):
            t1 = time.time()
            axioms = set()
            for p in preds:
                axioms |= pred_to_axioms[p]

            recursive_atoms = set()
            nonrecursive_atoms = set()
            true_atoms = set()
            for a in axioms:
                #check if the combination is actually valid, as sometimes a parameter type can depend
                #on a previous parameter (e.g. (typeof ?f) types)
                def filter_invalid(c):
                    return self.problem.predicates.get(a.predicate.name, c)
                
                if relevant:
                    gen = imap(lambda svar: svar.asLiteral(), relevant)
                else:
                    combinations = product(*map(lambda arg: list(self.problem.getAll(arg.type)), a.args))

                    gen = imap(lambda c: Literal(a.predicate, c, self.problem), ifilter(filter_invalid, combinations))
                    
                for atom in gen:
                    
                    if a.predicate in self.problem.nonrecursive:
                        nonrecursive_atoms.add(atom)
                    else:
                        recursive_atoms.add(atom)

                    svar = StateVariable.fromLiteral(atom)
                    if svar in self and self[svar] == types.TRUE:
                        true_atoms.add(atom)

            #print "collecting level %d (%d atoms):" % (level, len(recursive_atoms)+len(nonrecursive_atoms)), time.time()-t1
            
            changed = True
            while changed:
                t2 = time.time()
                changed = False
                open = (nonrecursive_atoms | recursive_atoms) - true_atoms
                for atom in open:
                    #t3 = time.time()
                    for ax in pred_to_axioms[atom.predicate]:
                        if ax.tryInstantiate(atom.args):
                            if getReasons:
                                vars = []
                                universal = []
                            else:
                                vars = None
                                universal = None
                            if self.isSatisfied(ax.condition, vars, universal):
                                true_atoms.add(atom)
                                changed = True
                                if getReasons:
                                    this_svar = StateVariable.fromLiteral(atom)
                                    for svar in vars:
                                        reasons[this_svar].add(svar)
                                    for param in universal:
                                        universalReasons[this_svar].add(param)
                                break
                            ax.uninstantiate()
                    #print "atom:", time.time()-t3

                nonrecursive_atoms = set()
                #print "iteration:", time.time()-t2
            #print "layer:", time.time()-t1
        
        #print "total:", time.time()-t0
                
        if getReasons:
            return State(itertools.chain(self.iterfacts(), [Fact(StateVariable.fromLiteral(a), types.TRUE) for a in true_atoms]), self.problem), reasons, universalReasons
        
        return State(itertools.chain(self.iterfacts(), [Fact(StateVariable.fromLiteral(a), types.TRUE) for a in true_atoms]), self.problem)
