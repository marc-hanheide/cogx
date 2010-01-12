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
    
    def copy(self, new_scope=None, copy_instance=False):
        return self.__class__()

    def free(self):
        def visitor(eff, results=[]):
            if isinstance(eff, predicates.Term):
                if eff.__class__ == predicates.FunctionTerm:
                    return sum(results, [])
                if isinstance(eff, predicates.VariableTerm):
                    return [eff.object]
                return []
            if isinstance(eff, LiteralCondition):
                return sum([t.visit(visitor) for t in eff.args], [])
            if isinstance(eff, list):
                return sum(results, [])
            if eff.__class__ == UniversalEffect:
                vars = results[0]
                return [p for p in vars if p not in eff]
            if eff.__class__ == ConditionalEffect:
                return results + list(eff.condition.free())
        return set(self.visit(visitor))
    
    def pddl_str(self, instantiated=True):
        def printVisitor(eff, results=[]):
            if eff.__class__ == SimpleEffect:
                s = "(%s %s)" % (eff.predicate.name, " ".join(a.pddl_str(instantiated) for a in eff.args))
                if eff.negated:
                    return "(not %s)" % s
                return s
            if isinstance(eff, list):
                return "(and %s)" % " ".join(results)
            if eff.__class__ == UniversalEffect:
                args = " ".join(sorted(cond.iterkeys()))
                return "(forall (%s) %s)" % (args, " ".join(results))
            if eff.__class__ == ConditionalEffect:
                return "(when (%s) %s)" % (eff.condition.pddl_str(), " ".join(results))
        return self.visit(printVisitor)
    
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
        
        elif first.token.string == "probabilistic":
            return ProbabilisticEffect.parse(it.reset(), scope, timedEffects, onlySimple)

        elif first.token.string == "assign-probabilistic":
            return ProbabilisticEffect.parse_assign(it.reset(), scope)
        
        else:
            if timedEffects:
                import durative
                return durative.TimedEffect.parse(it.reset(), scope)
            else:
                return SimpleEffect.parse(it.reset(), scope)
    
class UniversalEffect(scope.Scope, Effect):
    def __init__(self, args, effects, parentScope):
        scope.Scope.__init__(self, args, parentScope)
        self.args = args
        self.effects = effects

    def visit(self, fn):
        return fn(self, [e.visit(fn) for e in self.effects])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.parent
            
        cp = UniversalEffect([predicates.Parameter(a.name, a.type) for a in self.args], [], new_scope)
        for arg in cp.args:
            if isinstance(arg.type, types.ProxyType):
                if copy_instance and arg.type.parameter.isInstantiated():
                    arg.type = arg.type.effectiveType()
                else:
                    arg.type = types.ProxyType(cp[arg.type.parameter])
                    
        if new_parts:
            cp.effects = new_parts
            for e in cp.effects:
                e.set_scope(self)
        else:
            cp.effects = [e.copy(cp, copy_instance=copy_instance) for e in self.effects]
        return cp

    def set_scope(self, new_scope):
        self.parent = new_scope
        for e in self.effects:
            e.set_scope(self)
    
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
    
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if new_parts:
            return ProbabilisticEffect([(p, [eff.copy(new_scope, copy_instance=copy_instance) for eff in e]) for p,e in self.new_parts])
        else:
            return ProbabilisticEffect([(p, [eff.copy(new_scope, copy_instance=copy_instance) for eff in e]) for p,e in self.effects])

    def set_scope(self, new_scope):
        for p, e in self.effects:
            e.set_scope(new_scope)
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.effects == other.effects

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse_assign(it, scope):
        first = it.get("assign-probabilistic").token

        head = it.get(list, "function")

        values = []
        remaining_values = []
        psum = 0
        while True:
            try:
                elem = it.next()
            except StopIteration:
                break
            
            if elem.isTerminal():
                try:
                    prob = float(elem.token.string)
                except:
                    remaining_values.append(elem)
                    continue

                psum += prob
                if psum > 1:
                    raise ParseError(elem.token, "Total probabilities exceed 1.0")

                values.append((prob, it.get()))

            else:
                remaining_values.append(elem)

        if remaining_values and psum < 1.0:
            remaining_p = (1.0-psum) / len(remaining_values)
            for elem in remaining_values:
                values.append((remaining_p, elem))

        effects = []
        assign = parser.Element(parser.Token("assign", first.line, first.file))
        for p, elem in values:
            elem = parser.Element(it.element.token, [assign, head, elem])
            effs = Effect.parse(iter(elem), scope)
            effects.append((p, effs))

        return [ProbabilisticEffect(effects)]

    @staticmethod
    def parse(it, scope, timedEffects=False, onlySimple=False):
        first = it.get("probabilistic")
        effects = []
        remaining_effects = []
        psum = 0
        while True:
            try:
                elem = it.next()
            except StopIteration:
                if remaining_effects and psum < 1.0:
                    remaining_p = (1.0-psum) / len(remaining_effects)
                    for effs in remaining_effects:
                        effects.append((remaining_p, effs))

                return [ProbabilisticEffect(effects)]
            
            if elem.isTerminal():
                try:
                    prob = float(elem.token.string)
                except:
                    raise UnexpectedTokenError(p_elem.token, "probability or effect")

                psum += prob
                if psum > 1:
                    raise ParseError(elem.token, "Total probabilities exceed 1.0")

                effs = Effect.parse(iter(it.get(list, "effect specification")), scope, timedEffects, onlySimple)
                effects.append((prob, effs))

            else:
                effs = Effect.parse(iter(elem), scope, timedEffects, onlySimple)
                remaining_effects.append(effs)
            
                

class ConditionalEffect(Effect):
    def __init__(self, condition, effects):
        self.condition = condition
        self.effects = effects
        
    def visit(self, fn):
        return fn(self, [e.visit(fn) for e in self.effects])
    
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if new_parts:
            return ConditionalEffect(self.condition.copy(new_scope), [e.copy(new_scope, copy_instance=copy_instance) for e in new_parts])
        else:
            return ConditionalEffect(self.condition.copy(new_scope, copy_instance=copy_instance), [e.copy(new_scope, copy_instance=copy_instance) for e in self.effects])
        
    def set_scope(self, new_scope):
        self.condition.set_scope(new_scope)
        for e in self.effects:
            e.set_scope(new_scope)

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

