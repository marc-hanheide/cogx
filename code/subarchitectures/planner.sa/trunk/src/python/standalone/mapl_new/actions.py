#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
from parser import ParseError
import mapltypes as types
import predicates, conditions, effects
from scope import Scope

class Action(Scope):
    def __init__(self, name, args, precondition, effects, domain, replan=None):
        Scope.__init__(self, args, domain)
        self.name = name
        self.args = args

        self.precondition = precondition
        self.replan = replan
        self.effects = effects
        if effects is None:
            self.effects = []

        if self.precondition:
            self.precondition.set_scope(self)
        if self.replan:
            self.replan.set_scope(self)
            
        for e in self.effects:
            e.set_scope(self)

    def instantiate(self, mapping):
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        Scope.instantiate(self, mapping)

    def to_pddl(self):
        str = ["(:action %s" % self.name]
        indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        args = [types.Parameter(p.name, p.type) for p in self.args]
        
        a = Action(self.name, args, None, [], newdomain)

        for arg in a.args:
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(a[arg.type.parameter])
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
            
        a.effects = [e.copy(a) for e in self.effects]

        return a
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        action = Action(name, params, None, [], scope)

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
                    if action.effects:
                        raise ParseError(next.token, "effects already defined.")
                    action.effects = effects.Effect.parse(iter(it.get(list, "effect")), action)
                    
                next = it.next()

        except StopIteration, e:
            pass
            
        return action
