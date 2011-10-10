#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError
from scope import Scope, SCOPE_EFFECT
import builtin
import conditions, predicates

class Effect(object):
    """This is an abstract base class for all effect objects."""
    
    def visit(self, fn):
        """Visit this Effect and all its children.

        Arguments:
        fn -- visitor function that will be called with the current
        Effect and a list of results of previous calls."""
        return fn(self, [])
    
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        """Return a deep copy of this Effect object

        Arguments:
        new_scope -- if not None, the copy will be created in this scope
        new_parts -- if not None, replace the content of this effect with those given in this argument.
        copy_instance -- if True, replace all instantiated Parameters with their instantiations"""
        return self.__class__()

    def set_scope(self, new_scope):
        """Set a new scope for this Effect and all its children

        Arguments:
        new_scope -- Scope object"""
        assert new_scope is None or isinstance(new_scope, Scope)
        self.scope = new_scope

    def get_scope(self):
        """Return the Scope obect that this Effect resides in."""
        return self.scope
    
    def free(self):
        """Return the set of free Parameters that are used in this Effect."""
        
        def visitor(eff, results=[]):
            if isinstance(eff, predicates.Term):
                if eff.__class__ == predicates.FunctionTerm:
                    return sum(results, [])
                if isinstance(eff, predicates.VariableTerm):
                    return [eff.object]
                return []
            if isinstance(eff, SimpleEffect):
                return sum([t.visit(visitor) for t in eff.args], [])
            if eff.__class__ == UniversalEffect:
                vars = results[0]
                return [p for p in vars if p not in eff.args]
            if eff.__class__ == ConditionalEffect:
                return sum(results, []) + list(eff.condition.free())
            return sum(results, [])
        return set(self.visit(visitor))
    
    def pddl_str(self, instantiated=True):
        """Return a pddl text representation of this Effect.
        
        Arguments:
        instantiated -- if True (which is the default) resolves
        instantiated Parameters before printing the string."""

        def printVisitor(eff, results=[]):
            if isinstance(eff, SimpleEffect):
                s = "(%s %s)" % (eff.predicate.name, " ".join(a.pddl_str(instantiated) for a in eff.args))
                if eff.negated:
                    s = "(not %s)" % s
                return s
            if isinstance(eff, ConjunctiveEffect):
                return "(and %s)" % " ".join(results)
            if eff.__class__ == UniversalEffect:
                args = " ".join(sorted(eff.iterkeys()))
                return "(forall (%s) %s)" % (args, results[0])
            if eff.__class__ == ConditionalEffect:
                return "(when (%s) %s)" % (eff.condition.pddl_str(), results[0])
            if eff.__class__ == ProbabilisticEffect:
                pstrs = ["%s %s" % (p.pddl_str(), e.pddl_str()) for p,e in eff.effects]
                return "(probabilistic %s)" % " ".join(pstrs)
            try:
                return eff.pddl_str_extra(results, instantiated=instantiated)
            except:
                assert False, "Class not handled: %s" % eff.__class__
        return self.visit(printVisitor)
    
    @staticmethod
    def parse(it, scope):
        for handler in scope.parse_handlers:
            if "Effect" in handler:
                eff = handler["Effect"](it.reset(), scope)
                if eff:
                    return eff
                
        it.reset()
        first = it.get(None, "effect specification")
        if first.token.string == "and":
            return ConjunctiveEffect.parse(it.reset(), scope)
        
        elif first.token.string == "forall":
            if scope.get_tag("only-simple-effects"):
                raise ParseError(first.token, "'forall' is not allowed here.")
            return UniversalEffect.parse(it.reset(), scope)
        
        elif first.token.string == "when":
            if scope.get_tag("only-simple-effects"):
                raise ParseError(first.token, "Conditional effects are not allowed here.")
            return ConditionalEffect.parse(it.reset(), scope)
        
        elif first.token.string == "probabilistic":
            return ProbabilisticEffect.parse(it.reset(), scope)

        elif first.token.string == "assign-probabilistic":
            return ProbabilisticEffect.parse_assign(it.reset(), scope)
        
        else:
            return SimpleEffect.parse(it.reset(), scope)

    def __ne__(self, other):
        return not self.__eq__(other)
            
class ConjunctiveEffect(Effect):
    """This class represents a list of zero or more Effects."""
    
    def __init__(self, effects, scope=None):
        """Create a new ConjunctiveEffect.

        Arguments:
        effects -- Effects that this ConjunctiveEffect contains
        scope -- Scope this Effect resides in.
        """
        self.parts = effects
        self.scope = scope
        
    def visit(self, fn):
        return fn(self, [e.visit(fn) for e in self.parts])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        if new_parts is None:
            return ConjunctiveEffect([e.copy(new_scope, copy_instance=copy_instance) for e in self.parts])
        elif new_scope:
            for p in new_parts:
                p.set_scope(new_scope)
                
        return ConjunctiveEffect(new_parts)

    def set_scope(self, new_scope):
        self.scope = new_scope
        for e in self.parts:
            e.set_scope(new_scope)
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.parts == other.parts

    def __hash__(self):
        return hash((self.__class__,) + tuple(self.parts))

    @staticmethod
    def join(effs, scope=None):
        """Join together a list of effects in a Conjunctive effect. If an element of the list
        is already a conjunctive effect, add its elements."""
        parts = []
        for eff in effs:
            if not eff:
                continue
            if isinstance(eff, ConjunctiveEffect):
                parts += eff.parts
            else:
                parts.append(eff)
            if scope is None and eff.get_scope() is not None:
                scope = eff.get_scope()
        return ConjunctiveEffect(parts, scope)
    
    @staticmethod
    def parse(it, scope):
        it.get("and")

        eff = ConjunctiveEffect([])
        for elem in it:
            eff.parts.append(Effect.parse(iter(elem), scope))
            
        return eff
    
class UniversalEffect(Scope, Effect):
    """This class represents a universal effect."""
    
    def __init__(self, args, effect, parentScope):
        """Create a new UniversalEffect

        Arguments:
        args -- List of Parameters over which the effect shall be quantified
        effect -- The Effect that shall be quantified over
        parent -- Scope this Effect resides in.
        """
        Scope.__init__(self, args, parentScope)
        self.args = args
        self.effect = effect

    def visit(self, fn):
        return fn(self, [self.effect.visit(fn)])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.parent
            
        cp = UniversalEffect([], None, new_scope)
        cp.args = cp.copy_args(self.args, copy_instance)

        if new_parts == []:
            new_parts = ConjunctiveEffect([])
        elif new_parts:
            cp.effect = new_parts[0]
            cp.effect.set_scope(cp)
        else:
            cp.effect = self.effect.copy(cp, copy_instance=copy_instance)
        return cp

    def set_scope(self, new_scope):
        Scope.set_parent(self, new_scope)
        self.effect.set_scope(self)

    def get_scope(self):
        return self.parent
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.args == other.args and self.effect == other.effect

    def __hash__(self):
        return hash((self.__class__, self.effect) + tuple(self.args))
    
    @staticmethod
    def parse(it, scope):
        it.get("forall")
        args = predicates.parse_arg_list(iter(it.get(list, "parameter list")), scope.types)
        eff = UniversalEffect(args, None, scope)
        
        eff.effect = Effect.parse(iter(it.get(list, "effect specification")), eff)

        return eff

class ProbabilisticEffect(Effect):
    """This class represents a probabilistic effect."""
    
    def __init__(self, effects, scope=None):
        """Create a new ProbabilisticEffect

        Arguments:
        effects -- A list of (probability, Effect) tuples that describe the possible Effects and their probability.
        scope -- Scope this Effect resides in.
        """
        self.effects = effects
        self.scope = scope
        # self.summed_effects = []
        # psum = 0
        # for p, eff in effects:
        #     self.summed_effects.append((psum, psum+p, eff))
        #     psum += p
        # assert psum <= 1

    # def getRandomEffect(self, seed=None):
    #     if seed is not None:
    #         random.seed(seed)
    #     s = random.random()
    #     for start, end, eff in self.summed_effects:
    #         if start <= s < end:
    #             return eff
    #     return None

    def get_effects(self, state):
        remaining_effects = []
        p_total = 0
        for p, e in self.effects:
            assert p_total <= 1.0
            if p is None:
                remaining_effects.append(e)
            else:
                p = state.evaluate_term(p).value if state else p.object.value
                p_total += p
                yield (p, e)

        if remaining_effects:
            p = (1.0-p_total) / len(remaining_effects)
            for e in remaining_effects:
                yield (p,e)
        
    def visit(self, fn):
        return fn(self, [(p, e.visit(fn)) for p,e in self.effects])
    
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        if new_parts == []:
            return ProbabilisticEffect([])
        parts = new_parts if new_parts else self.effects
        res_parts = []
        for p, e in parts:
            if p and copy_instance:
                p = p.copy_instance()
            elif p:
                p = p.copy(new_scope)
            res_parts.append((p, e.copy(new_scope, copy_instance=copy_instance)))
            
        return ProbabilisticEffect(res_parts)

    def set_scope(self, new_scope):
        self.scope = new_scope
        for p, e in self.effects:
            e.set_scope(new_scope)
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.effects == other.effects

    def __hash__(self):
        return hash((self.__class__,) + tuple(self.effects))

    @staticmethod
    def parse_assign(it, scope):
        first = it.get("assign-probabilistic").token

        assign = parser.Element(parser.Token("assign", first.line, first.file))
        head = it.get(list, "function")

        effects = []

        next_prob = None
        for elem in it:
            term = predicates.Term.parse(elem, scope)
            if not next_prob and term.get_type().equal_or_subtype_of(builtin.t_number):
                next_prob = term
            else:
                el2 = parser.Element(elem.token, [assign, head, elem])
                eff = Effect.parse(iter(el2), scope)
                effects.append((next_prob, eff))
                next_prob = None

        return ProbabilisticEffect(effects)

    @staticmethod
    def parse(it, scope):
        it.get("probabilistic")
        token_dict = {}
        parsed_elements = []

        for elem in it:
            try:
                pddl_elem = Effect.parse(iter(elem), scope)
            except ParseError:
                pddl_elem = predicates.Term.parse(elem, scope)

            parsed_elements.append(pddl_elem)
            token_dict[pddl_elem] = elem.token

        effects = []

        next_prob = None
        for elem in parsed_elements:
            if isinstance(elem, predicates.Term) and elem.get_type().equal_or_subtype_of(builtin.t_number):
                if next_prob:
                    raise UnexpectedTokenError(token_dict[elem], "effect")
                next_prob = elem
            elif isinstance(elem, Effect):
                effects.append((next_prob, elem))
                next_prob = None
            else:
                print elem
                raise UnexpectedTokenError(token_dict[elem], "probability or effect")
            
        return ProbabilisticEffect(effects)
                

class ConditionalEffect(Effect):
    """This class represents a conditional effect."""
    
    def __init__(self, condition, effect, scope=None):
        """Create a new ConditionalEffect

        Arguments:
        condition -- Condition object that must evaluate to True for this effect to be applied
        effect -- The Effect object the condition applies to.
        scope -- Scope this Effect resides in.
        """
        self.condition = condition
        self.effect = effect
        self.scope = scope
        
    def visit(self, fn):
        return fn(self, [self.effect.visit(fn)])
    
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        if new_parts == []:
            return ConditionalEffect(self.condition.copy(new_scope), ConjunctiveEffect([], new_scope), new_scope)
        elif new_parts:
            return ConditionalEffect(self.condition.copy(new_scope), new_parts[0].copy(new_scope, copy_instance=copy_instance), new_scope)
        else:
            return ConditionalEffect(self.condition.copy(new_scope, copy_instance=copy_instance), self.effect.copy(new_scope, copy_instance=copy_instance), new_scope)
        
    def set_scope(self, new_scope):
        self.scope = new_scope
        self.condition.set_scope(new_scope)
        self.effect.set_scope(new_scope)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.condition == other.condition and self.effect == other.effect

    def __hash__(self):
        return hash((self.__class__, self.condition, self.effect))
        
    @staticmethod
    def parse(it, scope):
        it.get("when")
        condition = conditions.Condition.parse(iter(it.get(list, "condition")), scope)
        #scope.set_tag("only-simple-effects", True)
        effects = Effect.parse(iter(it.get(list, "effect specification")), scope)
        #scope.set_tag("only-simple-effects", False)
            
        return ConditionalEffect(condition, effects, scope)

class SimpleEffect(predicates.Literal, Effect):
    """This class represents an effect that sets/unsets a literal or assigns values to fluents."""
    
    def __str__(self):
        return "Effect: %s" % predicates.Literal.__str__(self)

    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token

        literal = predicates.Literal.parse(it.reset(), scope, function_scope=SCOPE_EFFECT)

        if literal.predicate in builtin.assignment_ops + builtin.numeric_ops and literal.negated:
            raise ParseError(first, "Can't negate fluent assignments.")

        if literal.predicate == builtin.equals:
            raise ParseError(first, "Can't use '=' in effects, please use 'assign' instead.")

        return SimpleEffect(literal.predicate, literal.args, scope, literal.negated)

