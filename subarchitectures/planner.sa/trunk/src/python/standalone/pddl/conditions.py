#! /usr/bin/env python
# -*- coding: latin-1 -*-

from parser import UnexpectedTokenError
import mapltypes as types
from scope import Scope
import predicates

class Condition(object):
    def visit(self, fn):
        return fn(self, [])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        return self.__class__()

    def set_scope(self, new_scope):
        pass

    def free(self):
        def visitor(cond, results=[]):
            if isinstance(cond, predicates.Term):
                if cond.__class__ == predicates.FunctionTerm:
                    return sum(results, [])
                if isinstance(cond, predicates.VariableTerm):
                    return [cond.object]
                return []
            if isinstance(cond, LiteralCondition):
                return sum([t.visit(visitor) for t in cond.args], [])
            if isinstance( cond, JunctionCondition):
                return sum(results, [])
            if isinstance(cond, QuantifiedCondition):
                vars = results[0]
                return [p for p in vars if p not in cond.args]
        return set(self.visit(visitor))

    def has_class(self, _class):
        def visitor(cond, results=[]):
            if isinstance(cond, _class):
                return True
            return any(results)
        return self.visit(visitor)
    
    def __eq__(self, other):
        return self.__class__ == other.__class__

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.__class__)

    def pddl_str(self, instantiated=True):
        def printVisitor(cond, results=[]):
            if cond.__class__ == LiteralCondition:
                s = "(%s %s)" % (cond.predicate.name, " ".join(a.pddl_str(instantiated) for a in cond.args))
                if cond.negated:
                    return "(not %s)" % s
                return s
            if cond.__class__ == Conjunction:
                return "(and %s)" % " ".join(results)
            if cond.__class__ == Disjunction:
                return "(or %s)" % " ".join(results)
            if cond.__class__ == UniversalCondition:
                args = " ".join(sorted(cond.iterkeys()))
                return "(forall (%s) %s)" % (args, " ".join(results))
            if cond.__class__ == ExistentialCondition:
                args = " ".join(sorted(cond.iterkeys()))
                return "(exists (%s) %s)" % (args, " ".join(results))
        return self.visit(printVisitor)
    
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
                if part.is_terminal():
                    raise UnexpectedTokenError(part.token, "condition")
                parts.append(Condition.parse(iter(part), scope))
            if tag == "and":
                return Conjunction(parts)
            else:
                return Disjunction(parts)
        elif tag == "imply":
            part1 = it.get(list, "condition")
            part2 = it.get(list, "condition")
            it.no_more_tokens()
            condition = Condition.parse(iter(part1), scope)
            implication = Condition.parse(iter(part1), scope)
            return Disjunction([condition.negate(), implication])
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

    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_parts:
            new_parts = self.parts
        return self.__class__([ p.copy(new_scope, copy_instance=copy_instance) for p in new_parts])

    def set_scope(self, new_scope):
        for p in self.parts:
            p.set_scope(new_scope)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and all(map(lambda a,b: a==b, self.parts, other.parts))
    
    def __hash__(self):
        return hash((self.__class__, ) + tuple(self.parts))
    
    
class Conjunction(JunctionCondition):
    def negate(self):
        neg_parts = [c.negate() for c in self.parts]
        return Disjunction(neg_parts)

class Disjunction(JunctionCondition):
    def negate(self):
        neg_parts = [c.negate() for c in self.parts]
        return Conjunction(neg_parts)

class QuantifiedCondition(Condition, Scope):
    def __init__(self, args, condition, parent):
        Scope.__init__(self, args, parent)
        self.args = args
        self.condition = condition

    def visit(self, fn):
        return fn(self, [self.condition.visit(fn)])
    
    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_scope:
            new_scope = self.parent
            
        cp = self.__class__([predicates.Parameter(a.name, a.type) for a in self.args], None, new_scope)
        for arg in cp.args:
            if isinstance(arg.type, types.ProxyType):
                if copy_instance and arg.type.parameter.is_instantiated():
                    arg.type = arg.type.effective_type()
                else:
                    arg.type = types.ProxyType(cp[arg.type.parameter])

        if new_parts:
            cp.condition = new_parts[0]
            cp.condition.set_scope(cp)
        else:
            cp.condition = self.condition.copy(cp, copy_instance=copy_instance)
        return cp
    
    def set_scope(self, new_scope):
        Scope.set_parent(self, new_scope)
        self.condition.set_scope(self)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.condition == other.condition and all(map(lambda a,b: a==b, self.args, other.args))

    def __hash__(self):
        return hash((self.__class__, self.condition ) + tuple(self.args))
    
    @staticmethod
    def parse(it, scope, _class, parseFn=Condition.parse):
        variables = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types, scope)
        cond = _class(variables, None, scope)
        cond.condition = parseFn(iter(it.get(list, "condition")), cond)
        return cond

class UniversalCondition(QuantifiedCondition):
    def negate(self):
        neg = ExistentialCondition([types.Parameter(p.name, p.type) for p in self.args], None, self.parent)
        neg.condition = self.condition.negate()
        neg.condition.set_scope(neg)
        return neg

class ExistentialCondition(QuantifiedCondition):
    def negate(self):
        neg = UniversalCondition([types.Parameter(p.name, p.type) for p in self.args], None, self.parent)
        neg.condition = self.condition.negate()
        neg.condition.set_scope(neg)
        return neg

class LiteralCondition(predicates.Literal, Condition):
    def __init__(self, predicate, args, scope=None, negated=False):
        predicates.Literal.__init__(self, predicate, args, scope, negated)
    
    @staticmethod
    def parse(it, scope):
        literal = predicates.Literal.parse(it,scope)
        return LiteralCondition(literal.predicate, literal.args, literal.negated)


