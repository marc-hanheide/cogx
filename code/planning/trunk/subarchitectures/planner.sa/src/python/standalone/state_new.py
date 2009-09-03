#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools
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

def get_svars_in_term(args, state=None):
    result = []
    relVars = []
    for arg in args:
        if isinstance(arg, VariableTerm):
            assert arg.object.isInstantiated()
            arg = arg.copyInstance()

        if isinstance(arg, ConstantTerm):
            result.append(arg.object)
        elif state is not None:
            svar, total = StateVariable.svarsInTerm(arg, state)
            if not svar in state:
                raise Exception("couldn't create state variable, %s is not in state." % str(arg))
            value = state[svar]
            result.append(value)
            relVars += total
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    return result, relVars

class StateVariable(object):
    def __init__(self, function, args, modality=None, modal_args=[]):
        self.function = function
        assert len(function.args) == len(args)
        for a, fa in zip(args, function.args):
            assert a.isInstanceOf(fa.type), "%s not of type %s" % (str(a), str(fa))
        self.args = args

        self.modality = modality
        self.modal_args = modal_args

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
        return self.__class__ == other.__class__ and self.function == other.function and all(map(lambda a,b: a==b, self.args, other.args)) and self.modality == other.modality and all(map(lambda a,b: a==b, self.modal_args, other.modal_args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.function,self.modality)+ tuple(self.args)+tuple(self.modal_args))

    @staticmethod
    def fromTerm(term, state=None):
        assert isinstance(term, FunctionTerm)
        args = instantiate_args(term.args, state)
        return StateVariable(term.function, args)

    @staticmethod
    def svarsInTerm(term, state=None):
        assert isinstance(term, FunctionTerm)
        args, vars = get_svars_in_term(term.args, state)
        svar = StateVariable(term.function, args)
        vars.append(svar)
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

    def iterfacts(self):
        return itertools.imap(lambda tup: Fact.fromTuple(tup), self.iteritems())

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
        facts = [Fact.fromLiteral(i) for i in problem.init]
        return State(facts, problem)
        
    def factsFromCondition(self, cond):
        return Fact.fromCondition(cond, self)

    def isSatisfied(self, cond, relevantVars=None):
        def instantianteAndCheck(cond, params):
            cond.instantiate(dict(zip(cond.args, params)))
            result = checkConditionVisitor(cond.condition)
            cond.uninstantiate()
            return result

        def allFacts(iterable):
            newfacts = []
            for result, facts in iterable:
                if not result:
                    return False, []
                newfacts += facts
            return True, newfacts

        def anyFacts(iterable):
            newfacts = []
            for result, facts in iterable:
                if result:
                    return True, facts
            return False, []
        
        def checkConditionVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                fact = Fact.fromLiteral(cond, self)
                result = fact in self
                if cond.negated:
                    result = not result
                    
                relVars = []
                if relevantVars is not None and result:
                    relVars.append(fact.svar)
                    for arg in cond.args:
                        if isinstance(arg, FunctionTerm):
                            _, vars = StateVariable.svarsInTerm(arg, self)
                            relVars += vars
                            
                return result, relVars

            elif isinstance(cond, conditions.Truth):
                return True, []
            elif isinstance(cond, conditions.Falsity):
                return False, []
            elif isinstance(cond, conditions.Conjunction):
                if not cond.parts:
                    return True, []

                return allFacts(itertools.imap(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.Disjunction):
                return anyFacts(itertools.imap(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                combinations = product(*map(lambda a: self.problem.getAll(a.type), cond.args))
                if isinstance(cond, conditions.UniversalCondition):
                    return allFacts(itertools.imap(lambda c: instantianteAndCheck(cond, c), combinations))
                elif isinstance(cond, conditions.ExistentialCondition):
                    return anyFacts(itertools.imap(lambda c: instantianteAndCheck(cond, c), combinations))

        result, svars = checkConditionVisitor(cond)
        if relevantVars is not None:
            relevantVars += svars
        return result

    def getRelevantVars(self, cond):
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
                return sum(itertools.imap(dependenciesVisitor, cond.parts), [])
            elif isinstance(cond, conditions.QuantifiedCondition):
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
    
    def applyEffect(self, effect):
        if isinstance(effect, effects.UniversalEffect):
            combinations = product(*map(lambda a: self.problem.getAll(a.type), effect.args))
            for params in combinations:
                effect.instantiate(zip(effect.args, params))
                for eff in effect.effects:
                    self.applyEffect(eff)
                effect.uninstantiate()
                
        elif isinstance(effect, effects.ConditionalEffect):
            if self.isSatisfied(effect.condition):
                for eff in effect.effects:
                    self.applyEffect(eff)

        else:
            self.set(Fact.fromEffect(effect))

    def getExtendedState(self, svars=[], getReasons=False):
        """Evaluate all axioms neccessary to instantiate the variables in 'svars'."""

        def getDependencies(svar, derived):
            open = set([svar])
            closed = set()
            while open:
                sv = open.pop()
                closed.add(sv)
                for ax in self.problem.axioms:
                    if ax.predicate == sv.getPredicate():
                        ax.instantiate(sv.getArgs())
                        for dep in self.getRelevantVars(ax.condition):
                            if dep.getPredicate() in derived and dep not in closed:
                                open.add(dep)
            return closed
                
                        
        pred_to_axioms = defaultdict(set)
        for a in self.problem.axioms:
            pred_to_axioms[a.predicate].add(a)
        derived = pred_to_axioms.keys()

        relevant = set()
        for s in svars:
            if s.getPredicate() in derived:
                relevant |= getDependencies(s, derived)

        if getReasons:
            reasons = defaultdict(set)

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
                if relevant:
                    gen = itertools.imap(lambda svar: svar.asLiteral(), relevant)
                else:
                    combinations = product(*map(lambda arg: list(self.problem.getAll(arg.type)), a.args))
                    gen = itertools.imap(lambda c: Literal(a.predicate, c, self.problem), combinations)
                    
                for atom in gen:
                    #typecheck for modal predicates:
                    #if any(map(lambda a: isinstance(a.type, types.FunctionType), atom.args))
                    #print "here:", map(str, c)
                    #atom = Literal(a.predicate, c, self.problem)
                    if a.predicate in self.problem.nonrecursive:
                        nonrecursive_atoms.add(atom)
                    else:
                        recursive_atoms.add(atom)

                    svar = StateVariable.fromLiteral(atom)
                    if svar in self and self[svar] == types.TRUE:
                        true_atoms.add(atom)

#            print "collecting level %d (%d atoms):" % (level, len(recursive_atoms)+len(nonrecursive_atoms)), time.time()-t1
            
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
                            else:
                                vars = None
                            if self.isSatisfied(ax.condition, vars):
                                true_atoms.add(atom)
                                changed = True
                                if getReasons:
                                    for svar in vars:
                                        reasons[StateVariable.fromLiteral(atom)].add(self[svar])
                                break
                            ax.uninstantiate()
                    #print "atom:", time.time()-t3

                nonrecursive_atoms = set()
#                print "iteration:", time.time()-t2
#            print "layer:", time.time()-t1
        
#        print "total:", time.time()-t0
                
        if getReasons:
            return State(itertools.chain(self.iterfacts(), [Fact(StateVariable.fromLiteral(a), types.TRUE) for a in true_atoms]), self.problem), reasons
        
        return State(itertools.chain(self.iterfacts(), [Fact(StateVariable.fromLiteral(a), types.TRUE) for a in true_atoms]), self.problem)
