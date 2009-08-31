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
            assert arg.object.isInstantiated()
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

    def __str__(self):
        return "%s(%s)" % (self.function.name, " ".join(map(lambda a: a.name, self.args)))

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
    
class State(dict):
    def __init__(self, facts=[], prob=None):
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

    def isSatisfied(self, cond):
        def getAll(type):
            for obj in itertools.chain(self.problem.objects, self.problem.constants):
                if obj.isInstanceOf(type):
                    yield obj

        def instantianteAndCheck(cond, params):
            cond.instantiate(dict(zip(cond.args, params)))
            result = checkConditionVisitor(cond.condition)
            cond.uninstantiate()
            return result
                    
        def checkConditionVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                return Fact.fromLiteral(cond, self) in self
            elif isinstance(cond, conditions.Truth):
                return True
            elif isinstance(cond, conditions.Falsity):
                return False
            elif isinstance(cond, conditions.Conjunction):
                if not cond.parts:
                    return True
                return all(map(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.Disjunction):
                return any(map(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                combinations = product(*map(lambda a: getAll(a.type), cond.args))
                if isinstance(cond, conditions.UniversalCondition):
                    return all(itertools.imap(lambda c: instantianteAndCheck(cond, c), combinations))
                elif isinstance(cond, conditions.ExistentialCondition):
                    return any(itertools.imap(lambda c: instantianteAndCheck(cond, c), combinations))

        return checkConditionVisitor(cond)

    def applyEffect(self, effect):
        if isinstance(effect, effects.UniversalEffect):
            combinations = product(*map(lambda a: getAll(a.type), effect.args))
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

    def getExtendedState(self, svars=[]):
        """Evaluate all axioms neccessary to instantiate the variables in 'svars'."""

        def getAll(type):
            if isinstance(type, types.FunctionType):
                for func in self.problem.functions:
                    if func.builtin:
                        continue
                    if types.FunctionType(func.type).equalOrSubtypeOf(type):
                        combinations = product(*map(lambda a: getAll(a.type), func.args))
                        for c in combinations:
                            yield FunctionTerm(func, c, self.problem)
            else:
                for obj in itertools.chain(self.problem.objects, self.problem.constants):
                    if obj.isInstanceOf(type):
                        yield obj
                        
        pred_to_axioms = defaultdict(set)
        for a in self.problem.axioms:
            pred_to_axioms[a.predicate].add(a)

        t0 = time.time()
        #all_axiom_preds = set([a.predicate for a in self.problem.axioms])
        all_axiom_atoms = set()
        true_atoms = set()
        for a in self.problem.axioms:
            combinations = product(*map(lambda arg: getAll(arg.type), a.args))
            for c in combinations:
                atom = Literal(a.predicate, c, self.problem)
                all_axiom_atoms.add(atom)
                
                svar = StateVariable.fromLiteral(atom)
                if svar in self and self[svar] == types.TRUE:
                    true_atoms.add(atom)
                    
        #print "collecting:", time.time()-t0
        
        changed = True
        while changed:
            t1 = time.time()
            changed = False
            open = all_axiom_atoms - true_atoms
            for atom in open:
                t2 = time.time()
                for ax in pred_to_axioms[atom.predicate]:
                    if all(map(lambda a,p: a.isInstanceOf(p.type), atom.args, ax.args)):
                        ax.instantiate(atom.args)
                        if self.isSatisfied(ax.condition):
                            true_atoms.add(atom)
                            changed = True
                            break
                #print "atom:", time.time()-t2
                    
            #print "iteration:", time.time()-t1
        
        #print "total:", time.time()-t0
        return State(itertools.chain(self.iterfacts(), [Fact(StateVariable.fromLiteral(a), types.TRUE) for a in true_atoms]), self.problem)
