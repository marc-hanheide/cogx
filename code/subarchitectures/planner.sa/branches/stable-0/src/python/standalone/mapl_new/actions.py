#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
from parser import ParseError
import mapltypes as types
import predicates, conditions, effects
from scope import Scope

class Action(Scope):
    def __init__(self, name, agents, args, vars, precondition, replan, effects, domain):
        Scope.__init__(self, agents+args+vars, domain)
        self.name = name
        self.agents = agents
        self.args = args
        self.vars = vars

        self.precondition = precondition
        self.replan = replan
        self.effects = effects

    def instantiate(self, mapping):
        if not isinstance(mapping, dict):
            mapping = dict([(param.name, c) for (param, c) in zip(self.agents+self.args+self.vars, mapping)])
        Scope.instantiate(self, mapping)

    def to_pddl(self):
        str = ["(:action %s" % self.name]
        indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        agents = [types.Parameter(p.name, p.type) for p in self.agents]
        args = [types.Parameter(p.name, p.type) for p in self.args]
        vars = [types.Parameter(p.name, p.type) for p in self.vars]
        
        a = Action(self.name, agents, args, vars, None, None, [], newdomain)

        for arg in itertools.chain(a.args, a.vars):
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
        it.get(":agent")
        agent = predicates.parseArgList(iter(it.get(list, "agent parameter")), scope.types)
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parseArgList(iter(it.get(list, "variables")), scope.types)
            next = it.get()
        else:
            variables = []

        action = Action(name, agent, params, variables, None, None, None, scope)
        

        if next.token.string == ":precondition":
            action.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), action)
            next = it.get()
        else:
            precondition = None
            
        if next.token.string == ":replan":
            action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
            it.get(":effect")
        else:
            replan = None
            next.token.checkKeyword(":effect")
            
        action.effects = effects.Effect.parse(iter(it.get(list, "effect")), action)
        return action

class DurationConstraint(object):
    def __init__(self, term, timeSpecifier):
        self.term = term
        self.timeSpecifier = timeSpecifier

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.term == other.term and self.timeSpecifier == other.timeSpecifier

    def __ne__(self, other):
        return not __eq__(self, other)

    @staticmethod
    def parse(it, scope):
        try:
            first = it.next().token
        except StopIteration:
            return []
        if first.string == "and":
            result = []
            for elem in it:
                result += DurationConstraint.parse(iter(elem), scope)
            return result
        
        time = None
        if first.string == "at":
            time = it.get("terminal").token
            if not time.string in ("start", "end"):
                raise UnexpectedTokenError(time, "'start' or 'end'")
            time = time.string
            it = iter(it.get(list, "duration constraint"))
            first = it.next().token

        if first.string != "=":
            raise UnexpectedTokenError(first, "=")
        it.get("?duration")
        
        term = predicates.Term.parse(it, scope)
        if not term.getType().equalOrSubtypeOf(types.numberType):
            raise ParseError(first, "Duration must be a number, not %s." % str(term.getType()))
        
        it.noMoreTokens()
        
        return [DurationConstraint(term, time)]
            
        
class DurativeAction(Action):
    def __init__(self, name, agents, args, vars, duration, precondition, replan, effects, domain):
        Action.__init__(self, name, agents, args, vars, precondition, replan, effects, domain)
        self.duration = duration

    def copy(self, newdomain=None):
        a = Action.copy(self, newdomain)
        a.__class__ = DurativeAction
        a.duration = [DurationConstraint(a.lookup([d.term])[0], d.timeSpecifier) for d in self.duration]
        return a
       
    @staticmethod
    def parse(it, scope):
        it.get(":durative-action")
    
        name = it.get().token.string
        it.get(":agent")
        agent = predicates.parseArgList(iter(it.get(list, "agent parameter")), scope.types)
        it.get(":parameters")
        params = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
        next = it.get()
        
        if next.token.string == ":variables":
            variables = predicates.parseArgList(iter(it.get(list, "variables")), scope.types)
            next = it.get()
        else:
            variables = []
            
        action =  DurativeAction(name, agent, params, variables, None, None, None, None, scope)
        
        next.token.checkKeyword(":duration")
        action.duration = DurationConstraint.parse(iter(it.get(list, "duration constraint")), action)

        next = it.get()
        if next.token.string == ":condition":
            action.precondition = conditions.TimedCondition.parse(iter(it.get(list, "condition")), action)
            next = it.get()
        else:
            precondition = None
            
        if next.token.string == ":replan":
            action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
            it.get(":effect")
        else:
            replan = None
            next.token.checkKeyword(":effect")
            
        action.add(types.TypedObject("?duration", types.numberType))
        action.effects = effects.Effect.parse(iter(it.get(list, "effect")), action, timedEffects=True)
        return action
