#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError
import mapltypes as types
import scope, conditions, predicates
import random

class Effect(object):
    def visit(self, fn):
        return fn(self, [])
    
    def copy(self, new_scope=None):
        return self.__class__()
    
    @staticmethod
    def parse(it, scope, timedEffects=False, onlySimple=False):
        first = it.get(None, "effect specification")
        effects = []
        if first.token.string == "and":
            for elem in it:
                effects += Effect.parse(iter(elem), scope, timedEffects, onlySimple)
            return effects
        
        elif not onlySimple and first.token.string == "forall":
            return UniversalEffect.parse(it.reset(), scope, timedEffects)
        
        elif not onlySimple and first.token.string == "when":
            return ConditionalEffect.parse(it.reset(), scope, timedEffects)
        
        elif not onlySimple and first.token.string == "probabilistic":
            return ProbabilisticEffect.parse(it.reset(), scope, timedEffects)
        
        else:
            if timedEffects:
                return TimedEffect.parse(it.reset(), scope)
            else:
                return SimpleEffect.parse(it.reset(), scope)
    
class UniversalEffect(scope.Scope, Effect):
    def __init__(self, args, effects, parentScope):
        scope.Scope.__init__(self, args, parentScope)
        self.args = args
        self.effects = effects

    def visit(self, fn):
        return fn(self, [e.visit(fn) for e in self.effects])

    def copy(self, new_scope=None):
        if not new_scope:
            new_scope = self.parent
            
        cp = UniversalEffect([predicates.Parameter(a.name, a.type) for a in self.args], [], new_scope)
        cp.effects = [e.copy(cp) for e in self.effects]
        return cp
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.args == other.args and self.effects == other.effects

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope, timedEffects=False):
        first = it.get("forall")
        args = predicates.parseArgList(iter(it.get(list, "parameter list")), scope.types)
        eff = UniversalEffect(args, [], scope)
        
        eff.effects = Effect.parse(iter(it.get(list, "effect specification")), eff, timedEffects)

        return [eff]

class ProbabilisticEffect(Effect):
    def __init__(self, effects):
        self.effects = effects
        self.summed_effects = []
        psum = 0
        for p, eff in effects:
            self.summed_effects.append((psum, psum+p, eff))
            psum += p
        assert psum <= 1

    def getRandomEffect(self, seed=None):       
        if seed is not None:
            random.seed(seed)
        s = random.random()
        for start, end, eff in self.summed_effects:
            if start <= s < end:
                return eff
        return []
        
    def visit(self, fn):
        return fn(self, [(p, e.visit(fn)) for p,e in self.effects])
    
    def copy(self, new_scope=None):
        return ProbabilisticEffect([(p, e.copy(new_scope)) for p,e in self.effects])

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.effects == other.effects

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope, timedEffects=False, onlySimple=False):
        first = it.get("probabilistic")
        effects = []
        psum = 0
        while True:
            try:
                p_elem = it.next()
            except StopIteration:
                return [ProbabilisticEffect(effects)]
            
            if not p_elem.isTerminal():
                raise UnexpectedTokenError(p_elem.token, "probability")
            try:
                prob = float(p_elem.token.string)
            except:
                raise UnexpectedTokenError(p_elem.token, "probability")

            if psum + prob > 1:
                raise ParseError(p_elem.token, "Total probabilities exceed 1.0")
                
            effs = Effect.parse(iter(it.get(list, "effect specification")), scope, timedEffects, onlySimple)
            effects.append((prob, effs))

class ConditionalEffect(Effect):
    def __init__(self, condition, effects):
        self.condition = condition
        self.effects = effects
        
    def visit(self, fn):
        return fn(self, [e.visit(fn) for e in self.effects])
    
    def copy(self, new_scope=None):
        return ConditionalEffect(self.condition.copy(new_scope), [e.copy(new_scope) for e in self.effects])

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.condition == other.condition and self.effects == other.effects

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope, timedEffects=False):
        first = it.get("when")
        condition = conditions.Condition.parse(iter(it.get(list, "condition")), scope)
        effects = Effect.parse(iter(it.get(list, "effect specification")), scope, timedEffects, onlySimple=True)
            
        return [ConditionalEffect(condition, effects)]

class SimpleEffect(predicates.Literal, Effect):
    def __str__(self):
        return "Effect: %s" % predicates.Literal.__str__(self)

    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token

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

        return [SimpleEffect(literal.predicate, literal.args, scope, literal.negated)]

class TimedEffect(SimpleEffect):
    def __init__(self, predicate, args, timeSpec, scope=None, negated=False):
        predicates.Literal.__init__(self, predicate, args, scope, negated)
        self.time = timeSpec
        
    def copy(self, new_scope=None):
        return TimedEffect(self.predicate, self.args, self.time, new_scope, self.negated)

    def __str__(self):
        return "TimedEffect at %s: %s" %(self.time, predicates.Literal.__str__(self))
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and predicates.Literal.__eq__(self, other)

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token

        if first.string != "at":
            scope.predicates.add(predicates.change)
            eff = SimpleEffect.parse(it.reset(), scope)
            scope.predicates.remove(predicates.change)
            return eff

        timespec = it.get().token

        if timespec.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")
        
        ops = [predicates.assign, predicates.change]
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

        return [TimedEffect(literal.predicate, literal.args, timespec.string, scope, literal.negated)]
