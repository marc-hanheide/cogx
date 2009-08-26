#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError
import mapltypes as types
import scope, conditions, predicates

class Effect(object):
    
    @staticmethod
    def parse(it, scope, timedEffects=False):
        first = it.get(None, "effect specification")
        effects = []
        if first.token.string == "and":
            for elem in it:
                effects += Effect.parse(iter(elem), scope, timedEffects)
            return effects

        return ConditionalEffect.parse(it.reset(), scope, timedEffects)
        

class ConditionalEffect(scope.Scope, Effect):
    def __init__(self, condition, variables, effects, parentScope):
        scope.Scope.__init__(self, variables, parentScope)
        self.variables = variables
        self.condition = condition
        self.effects = effects

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.variables == other.variables and self.condition == other.condition and self.effects == other.effects

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope, timedEffects=False):
        first = it.get()
        variables = []
        newscope = scope
        if first.token.string == "forall":
            variables = predicates.parseArgList(it, scope.types)
            it = iter(it.get(list, "effect specification"))
            first = it.get()
            
        ceff = ConditionalEffect(None, variables, [], scope)

        if first.token.string == "when":
            condition = conditions.Condition.parse(iter(it.get(list, "condition")), ceff)
            it = iter(it.get(list, "effect specification"))
            ceff.condition = condition
            
        if variables or ceff.condition:
            newscope = ceff
            
        if timedEffects:
            effects = TimedEffect.parse(it.reset(), newscope)
        else:
            effects = SimpleEffect.parse(it.reset(), newscope)

        if variables or ceff.condition:
            ceff.effects = effects
            return [ceff]
        
        return effects

class SimpleEffect(predicates.Literal, Effect):
    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token
        effects = []
        if first.string == "and":
            for elem in it:
                effects += parseSimpleEffects(iter(elem), scope, timedEffects)
            return effects

        ops = [predicates.assign]
        if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
            ops += predicates.numericOps

        scope.predicates.add(ops)
        scope.predicates.remove(predicates.equals)
        literal = predicates.Literal.parse(it.reset(), scope)
        scope.predicates.remove(ops)
        scope.predicates.add(predicates.equals)

        if literal.predicate in ops and literal.negated:
            raise ParseError(first, "Can't negate fluent assignments.")

        if literal.predicate == predicates.equals:
            raise ParseError(first, "Can't use '=' in effects, please use 'assign' instead.")

        return [SimpleEffect(literal.predicate, literal.args, literal.negated)]

class TimedEffect(SimpleEffect):
    def __init__(self, predicate, args, timeSpec, negated=False):
        predicates.Literal.__init__(self, predicate, args, negated)
        self.time = timeSpec
    
    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token
        effects = []
        if first.string == "and":
            for elem in it:
                effects += parseSimpleEffects(iter(elem), scope, timedEffects)
            return effects

        if first.string != "at":
            return SimpleEffect(it.reset(), scope)

        timespec = it.get().token

        if timespec.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")
        
        ops = [predicates.assign]
        if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
            ops += predicates.numericOps

        scope.predicates.add(ops)
        scope.predicates.remove(predicates.equals)
        literal = predicates.Literal.parse(iter(it.get(list, "effect")), scope)
        scope.predicates.remove(ops)
        scope.predicates.add(predicates.equals)

        if literal.predicate in ops and literal.negated:
            raise ParseError(first, "Can't negate fluent assignments.")

        if literal.predicate == predicates.equals:
            raise ParseError(first, "Can't use '=' in effects, please use 'assign' instead.")

        return [TimedEffect(literal.predicate, literal.args, timespec.string, literal.negated)]
