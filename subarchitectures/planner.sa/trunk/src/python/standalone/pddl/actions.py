#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

from parser import ParseError
import mapltypes as types
import predicates, conditions, effects
from scope import Scope

class Action(Scope):
    def __init__(self, name, args, precondition, effect, domain, replan=None):
        assert effect != []
        Scope.__init__(self, args, domain)
        self.name = name
        self.args = args

        self.precondition = precondition
        self.replan = replan
        self.effect = effect

        if self.precondition:
            self.precondition.set_scope(self)
        if self.replan:
            self.replan.set_scope(self)
        if self.effect:
            self.effect.set_scope(self)

    def instantiate(self, mapping):
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        Scope.instantiate(self, mapping)

    def to_pddl(self):
        pass
        #str = ["(:action %s" % self.name]
        #indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        args = [types.Parameter(p.name, p.type) for p in self.args]
        
        a = Action(self.name, args, None, None, newdomain)

        for arg in a.args:
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(a[arg.type.parameter])
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        if self.effect:
            a.effect = self.effect.copy(a)

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

        except StopIteration, e:
            pass
            
        return action
