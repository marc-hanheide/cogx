#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
import mapltypes as types
import predicates, conditions, effects, actions

class Sensor(actions.Action):
    def __init__(self, name, agents, args, vars, precondition, sense, domain):
        actions.Action.__init__(self, name, agents, args, vars, precondition, None, None, domain)
        self.sense = sense

    def knowledge_effect(self):
        if isinstance(self.sense, predicates.Literal):
            term = self.sense.args[0]
        else:
            term = self.sense

        return effects.SimpleEffect(predicates.direct_knowledge, [predicates.VariableTerm(self.agents[0]), term])
        
    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.domain
            
        agents = [predicates.Parameter(p.name, p.type) for p in self.agents]
        args = [predicates.Parameter(p.name, p.type) for p in self.args]
        vars = [predicates.Parameter(p.name, p.type) for p in self.vars]
        
        a = self.__class__(self.name, agents, args, vars, None, None, newdomain)

        for arg in itertools.chain(a.args, a.vars):
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(a[arg.type.parameter])
                
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if isinstance(self.sense, predicates.Literal):
            a.sense = self.sense.copy(a)
        else:
            a.sense = predicates.FunctionTerm(self.sense.function, a.lookup(self.sense.args))

        return a
        
        
    @staticmethod
    def parse(it, scope):
        it.get(":sensor")
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

        sensor =  Sensor(name, agent, params, variables, None, None, scope)
        

        if next.token.string == ":precondition":
            sensor.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), sensor)
            next = it.get()
        else:
            precondition = None

        next.token.checkKeyword(":sense")
        j = iter(it.get(list, "function or literal"))
        first = j.get("terminal", "predicate or function").token
        
        if first.string in scope.predicates:
            sensor.sense = predicates.Literal.parse(j.reset(), sensor)
        elif first.string in scope.functions:
            term = predicates.FunctionTerm.parse(j.reset(), sensor)
            sensor.sense = term
        else:
            raise parser.UnexpectedTokenError(first, "predicate, function or literal")

        return sensor
