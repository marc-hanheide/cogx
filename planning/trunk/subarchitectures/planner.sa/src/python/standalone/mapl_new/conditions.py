#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError
import mapltypes as types
import scope, predicates

class Condition(object):
    def visit(self, fn):
        return fn(self, [])

    def copy(self):
        return self.__class__()

    def __eq__(self, other):
        return self.__class__ == other.__class__

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.__class__)
    
    @staticmethod
    def parse(it, scope):
        first = it.get()
        tag = first.token.string
        if tag == "not":
            cond = Condition.parse(iter(it.get(list, "condition")), scope)
            return cond.negate()
        elif tag == "and" or tag == "or":
            parts = []
            for part in it:
                if part.isTerminal():
                    raise UnexpectedTokenError(part.token, "condition")
                parts.append(Condition.parse(iter(part), scope))
            if tag == "and":
                return Conjunction(parts)
            else:
                return Disjunction(parts)
        elif tag == "forall":
            return QuantifiedCondition.parse(it, scope, UniversalCondition)
        elif tag == "exists":
            return QuantifiedCondition.parse(it, scope, ExistentialCondition)
        else:
            return LiteralCondition.parse(it.reset(), scope)

        raise NotImplementedError

class Falsity(Condition):
    def negate(self):
        return Truth()

class Truth(Condition):
    def negate(self):
        return Falsity()

class JunctionCondition(Condition):
    def __init__(self, parts):
        self.parts = parts

    def visit(self, fn):
        return fn(self, [p.visit(fn) for p in self.parts])

    def copy(self):
        return self.__class__([ p.copy() for p in self.parts])

    def __eq__(self, other):
        return self.__class__ == other.__class__ and all(map(lambda a,b: a==b, self.parts, other.parts))

    def __hash__(self):
        return hash((self.__class__, ) + tuple(self.parts))
    
    
class Conjunction(JunctionCondition):
    def negate(self):
        neg_parts = map(lambda c: c.negate(), self.parts)
        return Disjunction(neg_parts)

class Disjunction(JunctionCondition):
    def negate(self):
        neg_parts = map(lambda c: c.negate(), self.parts)
        return Conjunction(neg_parts)

class QuantifiedCondition(Condition, scope.Scope):
    def __init__(self, args, condition, parent):
        scope.Scope.__init__(self, args, parent)
        self.args = args
        self.condition = condition

    def visit(self, fn):
        return fn(self, [self.condition.visit(fn)])

    def copy(self):
        return self.__class__([a.copy() for a in self.args], self.condition.copy(), self.parent)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.condition == other.condition and all(map(lambda a,b: a==b, self.args, other.args))

    def __hash__(self):
        return hash((self.__class__, self.condition ) + tuple(self.args))
    
    @staticmethod
    def parse(it, scope, _class, parseFn=Condition.parse):
        variables = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
        cond = _class(variables, None, scope)
        cond.condition = parseFn(iter(it.get(list, "condition")), cond)
        return cond

class UniversalCondition(QuantifiedCondition):
    def negate(self):
        return ExistentialCondition(self.condition.negate())

class ExistentialCondition(QuantifiedCondition):
    def negate(self):
        return UniversalCondition(self.condition.negate())

class LiteralCondition(Condition, predicates.Literal):
    def __init__(self, predicate, args, negated=False):
        predicates.Literal.__init__(self, predicate, args, negated)

    def negate(self):
        return LiteralCondition(self.predicate, self.args, not self.negated)

    def copy(self):
        return LiteralCondition(self.predicate, self.args, self.negated)
    
    @staticmethod
    def parse(it, scope):
        literal = predicates.Literal.parse(it,scope)
        return LiteralCondition(literal.predicate, literal.args, literal.negated)


class TimedCondition(Condition):
    def __init__(self, time, condition):
        assert time in ("start", "end", "all")
        self.time = time
        self.condition = condition

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and self.condition == other.condition

    def __hash__(self):
        return hash((self.__class__, self.time, self.condition ))

    
    @staticmethod
    def parse(it, scope):
        first = it.get()
        tag = first.token.string
        if tag == "and":
            parts = []
            for part in it:
                if part.isTerminal():
                    raise UnexpectedTokenError(part.token, "condition")
                parts.append(TimedCondition.parse(iter(part), scope))
            return Conjunction(parts)
        elif tag == "forall":
            return QuantifiedCondition.parse(it, scope, UniversalCondition, TimedCondition.parse)

        if not tag in ("at", "over"):
            raise UnexpectedTokenError(first.token, "timed condition ('at' or 'over')")

        specifier = it.get().token
        if tag == "at" and specifier.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")

        if tag == "over" and specifier.string != "all":
            raise UnexpectedTokenError(time, "'all'")

        condition = Condition.parse(iter(it.get(list)), scope)
        
        return TimedCondition(specifier.string, condition)


