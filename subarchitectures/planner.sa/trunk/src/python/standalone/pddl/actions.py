#! /usr/bin/env python
# -*- coding: latin-1 -*-
from parser import ParseError, UnexpectedTokenError
import predicates, conditions, effects, builtin, visitors
from scope import Scope

class Action(Scope):
    """This class represents a PDDl action."""
    
    def __init__(self, name, args, precondition, effect, domain, replan=None):
        """Create a new PDDL action.

        Arguments:
        name -- name of the action
        args -- List of Parameters for this action
        precondition -- a Condition object
        effect -- an Effect object
        domain -- the Domain this action belongs to.
        replan -- Condition object for the replan condition (defaults to None)"""
        
        assert effect != []
        Scope.__init__(self, args, domain)
        self.name = name
        self.args = args

        self.precondition = precondition
        self.replan = replan
        self.effect = effect
        self.totalCostFound = False
        self.totalCostTerm = None

        if self.precondition:
            self.precondition.set_scope(self)
        if self.replan:
            self.replan.set_scope(self)
        if self.effect:
            self.effect.set_scope(self)

    def instantiate(self, mapping, parent=None):
        """Instantiate the Parameters of this action.

        Arguments:
        mapping -- either a dictionary from Parameters to TypedObjects
        or a list of TypedObjects. In the latetr case, the list is
        assumed to be in the order of the Action's Parameters."""
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        Scope.instantiate(self, mapping, parent)

    def to_pddl(self):
        pass
        #str = ["(:action %s" % self.name]
        #indent = len("(:action ")

    def get_total_cost(self):
        self.totalCostTerm = None
        tct = predicates.Term(builtin.total_cost,[])

        def visitor(eff, parts):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate == builtin.increase:
                    if eff.args[0] == tct:
                        self.totalCostTerm = eff.args[1]

        visitors.visit(self.effect, visitor)
        return self.totalCostTerm

    def set_total_cost(self, new_total_cost):
        self.totalCostFound = False
        self.totalCostTerm = None
        tct = predicates.Term(builtin.total_cost,[])

        @visitors.replace
        def visitor(eff, parts):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate == builtin.increase:
                    if eff.args[0] == tct:
                        self.totalCostFound = True
                        if not new_total_cost:
                            return False
                        new_cost_eff = effects.SimpleEffect(builtin.increase,[tct,new_total_cost], eff.scope)
                        return new_cost_eff

        self.effect = self.effect.visit(visitor)

        if not self.totalCostFound:
            new_cost_eff = effects.SimpleEffect(builtin.increase,[tct,new_total_cost], self.effect.scope)
            if isinstance(self.effect, effects.ConjunctiveEffect):
                self.effect.parts.append(new_cost_eff)
            else:
                effs = [self.effect,new_cost_eff]
                new_eff = effects.ConjunctiveEffect(effs,self.effect.scope)
                self.effect = new_eff


    def copy(self, newdomain=None):
        """Create a deep copy of this Action.

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        a = Action(self.name, [], None, None, newdomain)
        a.args = a.copy_args(self.args)
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        if self.effect:
            a.effect = self.effect.copy(a)

        return a

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent

        a = Action(self.name, [], None, None, newdomain)
        a.args = a.copy_args(self.args)
        return a
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        action = Action(name, params, None, None, scope)

        try:
            while True:
                if next.token.string == ":precondition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action)
                else:
                    raise UnexpectedTokenError(next.token)
                    
                next = it.next()

        except StopIteration:
            pass
            
        return action
