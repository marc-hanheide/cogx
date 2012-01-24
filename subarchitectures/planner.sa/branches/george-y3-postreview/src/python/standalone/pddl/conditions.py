#! /usr/bin/env python
# -*- coding: latin-1 -*-

from parser import UnexpectedTokenError
import mapltypes as types
from scope import Scope, SCOPE_CONDITION
import predicates

class Condition(object):
    """This is an abstract base class for all condition objects."""
    
    def visit(self, fn):
        """Visit this Condition and all its children.

        Arguments:
        fn -- visitor function that will be called with the current
        Condition and a list of results of previous calls."""
        return fn(self, [])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        """Return a deep copy of this Condition object

        Arguments:
        new_scope -- if not None, the copy will be created in this scope
        new_parts -- if not None, replace the content of this condition with those given in this argument.
        copy_instance -- if True, replace all instantiated Parameters with their instantiations"""
        return self.__class__()

    def set_scope(self, new_scope):
        """Set a new scope for this Condition and all its children

        Arguments:
        new_scope -- Scope object"""
        assert new_scope is None or isinstance(new_scope, Scope)
        self.scope = new_scope

    def get_scope(self):
        """Return the Scope object that this Condition resides in."""
        return self.scope
    
    def free(self):
        """Return the set of free Parameters that are used in this Condition."""
        
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
            return sum(results, [])
        return set(self.visit(visitor))

    def get_constants(self):
        """Return the set of objects/constants that are used in this Condition."""
        
        def visitor(cond, results=[]):
            if isinstance(cond, IntermediateCondition):
                return results[0]
            if isinstance (cond, PreferenceCondition):
                return results[0]
            if isinstance(cond, predicates.Term):
                if cond.__class__ == predicates.FunctionTerm:
                    return sum(results, [])
                if isinstance(cond, predicates.ConstantTerm):
                    return [cond.object]
                return []
            if isinstance(cond, LiteralCondition):
                return sum([t.visit(visitor) for t in cond.args], [])
            if isinstance( cond, JunctionCondition):
                return sum(results, [])
            if isinstance(cond, QuantifiedCondition):
                vars = results[0]
                return [p for p in vars if p not in cond.args]
            return sum(results, [])
        return set(self.visit(visitor))

    def has_class(self, _class):
        """Return True if any descendant of this Condition is of class '_class'.

        Arguments:
        _class -- Class object to check for."""
        
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
        """Return a pddl text representation of this Condition.
        
        Arguments:
        instantiated -- if True (which is the default) resolves
        instantiated Parameters before printing the string."""
        
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
                args = " ".join(sorted(map(str, cond.itervalues())))
                return "(forall (%s) %s)" % (args, " ".join(results))
            if cond.__class__ == ExistentialCondition:
                args = " ".join(sorted(map(str, cond.itervalues())))
                return "(exists (%s) %s)" % (args, " ".join(results))
            if cond.__class__ == PreferenceCondition:
                return "(preference %d %s)" % (cond.penalty, results[0])
            return str(cond)
        return self.visit(printVisitor)
    
    @staticmethod
    def parse(it, scope):
        for handler in scope.parse_handlers:
            if "Condition" in handler:
                cond = handler["Condition"](it.reset(), scope)
                if cond:
                    return cond
                
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
            implication = Condition.parse(iter(part2), scope)
            return Disjunction([condition.negate(), implication])
        elif tag == "forall":
            return QuantifiedCondition.parse(it, scope, UniversalCondition)
        elif tag == "exists":
            return QuantifiedCondition.parse(it, scope, ExistentialCondition)
        elif tag == "soft-goal" or tag == "preference":
            penalty = float(it.get().token.string)
            preferenceCond = it.get(list, "condition")
            cond = Condition.parse(iter(preferenceCond), scope)
            it.no_more_tokens()
            return PreferenceCondition(penalty, cond, scope)
        elif tag == "intermediate" or tag == "sometime":
            intermediateCond = it.get(list, "condition")
            cond = Condition.parse(iter(intermediateCond), scope)
            it.no_more_tokens()
            return IntermediateCondition(cond, scope)
        else:
            return LiteralCondition.parse(it.reset(), scope)

        raise NotImplementedError

class IntermediateCondition(Condition):
    def __init__(self, _cond, _scope = None):
        self.cond = _cond
        self.scope = _scope

    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        if not new_parts:
            new_parts = [self.cond.copy(new_scope=new_scope, copy_instance=copy_instance)]
        elif new_scope:
            new_parts[0].set_scope(new_scope)
                
        return IntermediateCondition(new_parts[0], new_scope)

    #TODO: negate!
    def negate(self):
        raise NotImplementedError

    def visit(self, fn):
        return fn(self, [self.cond.visit(fn)])

class PreferenceCondition(Condition):
    def __init__(self, _penalty, _cond, _scope = None):
        self.penalty = _penalty
        self.cond = _cond
        self.scope = _scope

    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        if not new_parts:
            new_parts = [self.cond.copy(new_scope=new_scope, copy_instance=copy_instance)]
        elif new_scope:
            new_parts[0].set_scope(new_scope)
            
        return PreferenceCondition(self.penalty, new_parts[0], new_scope)

    def negate(self):
        raise NotImplementedError

    def visit(self, fn):
        return fn(self, [self.cond.visit(fn)])

class Falsity(Condition):
    """This class represents an always false condition."""
    
    def negate(self):
        """Return a negated copy of this condition."""
        return Truth()

class Truth(Condition):
    """This class represents an always true condition."""

    def negate(self):
        """Return a negated copy of this condition."""
        return Falsity()

class JunctionCondition(Condition):
    """This is an abstract base class for conjunctions and disjunctions."""
    
    def __init__(self, parts, scope=None):
        """Create a new JunctionCondition.

        Arguments:
        parts -- Conditions that this JunctionCondition contains
        scope -- Scope this Condition resides in.
        """
        
        self.parts = parts
        self.scope = scope

    def visit(self, fn):
        return fn(self, [p.visit(fn) for p in self.parts])

    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_parts:
            new_parts = self.parts
        elif new_scope:
            for p in new_parts:
                p.set_scope(new_scope)
        # if not new_scope:
        #     new_scope = self.scope
        return self.__class__([ p.copy(new_scope, copy_instance=copy_instance) for p in new_parts], new_scope)

    def set_scope(self, new_scope):
        self.scope = new_scope
        for p in self.parts:
            p.set_scope(new_scope)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and all(map(lambda a,b: a==b, self.parts, other.parts))
    
    def __hash__(self):
        return hash((self.__class__, ) + tuple(self.parts))

    @staticmethod
    def join(conds, scope=None, clas=None):
        """Join together a list of copnditions in a Junction
        condition, whose type is defined by the first not None
        element. If an element of the list is already a condition of
        that type, add its elements."""
        parts = []
        for c in conds:
            if not c:
                continue
            if clas is None and isinstance(c, JunctionCondition):
                clas = type(c)
                parts += c.parts
            elif isinstance(c, clas):
                parts += c.parts
            else:
                parts.append(c)
            if scope is None and isinstance(c, clas) and c.get_scope() is not None:
                scope = c.get_scope()
        return clas(parts, scope)
    
    
class Conjunction(JunctionCondition):
    """This class represents a conjunction of zero or more Conditions."""
    
    def negate(self):
        neg_parts = [c.negate() for c in self.parts]
        return Disjunction(neg_parts)

    @staticmethod
    def join(conds, scope=None):
        return JunctionCondition.join(conds, scope, Conjunction)
        
    @staticmethod
    def new(cond):
        """If cond is not a conjunction, return a new one with cond as an element.
        Otherwise, return cond unmodified"""
        if isinstance(cond, Conjunction):
            return cond
        if cond is None:
            return Conjunction([])
        return Conjunction([cond], cond.get_scope())
        
class Disjunction(JunctionCondition):
    """This class represents a disjunction of zero or more Conditions."""

    @staticmethod
    def join(conds, scope=None):
        return JunctionCondition.join(conds, scope, Disjunction)
    
    def negate(self):
        neg_parts = [c.negate() for c in self.parts]
        return Conjunction(neg_parts)

class QuantifiedCondition(Condition, Scope):
    """This is an abstract base class for existential and universal conditions."""
    
    def __init__(self, args, condition, parent):
        """Create a new QuantifiedCondition

        Arguments:
        args -- List of Parameters over which the condition shall be quantified
        condition -- The Condition that shall be quantified over
        parent -- Scope this Condition resides in.
        """
        
        Scope.__init__(self, args, parent)
        self.args = args
        self.condition = condition

    def visit(self, fn):
        return fn(self, [self.condition.visit(fn)])

    def get_scope(self):
        return self.parent
    
    def copy(self, new_scope=None, new_parts = None, copy_instance=False):
        if not new_scope:
            new_scope = self.parent
            
        cp = self.__class__([], None, new_scope)
        cp.args = cp.copy_args(self.args, copy_instance)
        
        if new_parts:
            cp.condition = new_parts[0]
            cp.condition.set_scope(cp)
        else:
            cp.condition = self.condition.copy(cp, copy_instance=copy_instance)
        return cp
    
    def set_scope(self, new_scope):
        Scope.set_parent(self, new_scope)
        self.args = self.copy_args(self.args, False)
        for a in self.args:
            self.add(a)
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
    """This class represents a universally quantified condition."""
    
    def negate(self):
        neg = ExistentialCondition([types.Parameter(p.name, p.type) for p in self.args], None, self.parent)
        neg.condition = self.condition.negate()
        neg.condition.set_scope(neg)
        return neg

class ExistentialCondition(QuantifiedCondition):
    """This class represents an existentially quantified condition."""
    
    def negate(self):
        neg = UniversalCondition([types.Parameter(p.name, p.type) for p in self.args], None, self.parent)
        neg.condition = self.condition.negate()
        neg.condition.set_scope(neg)
        return neg

class LiteralCondition(predicates.Literal, Condition):
    """This class represents a Literal in a condition."""
    
    def __init__(self, predicate, args, scope=None, negated=False):
        predicates.Literal.__init__(self, predicate, args, scope, negated)
    
    @staticmethod
    def parse(it, scope):
        literal = predicates.Literal.parse(it,scope, function_scope=SCOPE_CONDITION)
        return LiteralCondition(literal.predicate, literal.args, scope, literal.negated)


