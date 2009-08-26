#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools
import mapl_new as mapl
from mapl_new.predicates import *
from mapl_new import problem, conditions, effects, types

def instantiate_args(args, state=None):
    result = []
    for arg in args:
        if isinstance(arg, ConstantTerm):
            if isinstance(arg.object, types.Parameter):
                assert arg.object.isInstantiated()
                result.append(arg.object.getInstance())
            else:
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
        assert value.isInstanceOf(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
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
        if isinstance(effect, effects.ConditionalEffect):
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
        return itertools.imap(lambda tup: Fact. fromTuple(tup), self.iteritems())

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
        facts = Fact.fromCondition(cond, self)
        return all(map(lambda f: f in self, facts))

    def applyEffects(self, effects):
        facts = sum([Fact.fromEffect(e) for e in effects], [])
        for f in facts:
            self.set(f)
        
    
