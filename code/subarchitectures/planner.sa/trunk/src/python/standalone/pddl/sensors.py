#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
import mapltypes as types
import predicates, conditions, effects, mapl

class Sensor(mapl.MAPLAction):
    def __init__(self, name, agents, args, vars, precondition, sense, domain):
        mapl.MAPLAction.__init__(self, name, agents, args, vars, precondition, None, None, domain)
        self.sense = sense
        self.effect = self.knowledge_effect()

    def knowledge_effect(self):
        term = self.get_term()
        if not term:
            return None
        return effects.SimpleEffect(mapl.direct_knowledge, [predicates.VariableTerm(self.agents[0]), term])

    def is_boolean(self):
        return isinstance(self.sense, predicates.Literal)

    def get_term(self):
        if self.is_boolean():
            return self.sense.args[0]
        return self.sense

    def get_value(self):
        if self.is_boolean():
            return self.sense.args[1]
        return None
    
    def copy(self, newdomain=None):
        a = super(type(self), self).copy(newdomain)
        a.__class__ = self.__class__

        if isinstance(self.sense, predicates.Literal):
            a.sense = self.sense.copy(a)
        else:
            a.sense = predicates.FunctionTerm(self.sense.function, a.lookup(self.sense.args))

        a.effect = a.knowledge_effect()

        return a
        
    @staticmethod
    def parse(it, scope):
        it.get(":sensor")
        name = it.get().token.string
        it.get(":agent")
        agent = predicates.parse_arg_list(iter(it.get(list, "agent parameter")), scope.types)
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parse_arg_list(iter(it.get(list, "variables")), scope.types)
            next = it.get()
        else:
            variables = []

        sensor =  Sensor(name, agent, params, variables, None, None, scope)
        

        if next.token.string == ":precondition":
            sensor.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), sensor)
            next = it.get()
        else:
            precondition = None

        next.token.check_keyword(":sense")
        j = iter(it.get(list, "function or literal"))
        first = j.get("terminal", "predicate or function").token
        
        if first.string in scope.predicates:
            sensor.sense = predicates.Literal.parse(j.reset(), sensor)
        elif first.string in scope.functions:
            term = predicates.FunctionTerm.parse(j.reset(), sensor)
            sensor.sense = term
        else:
            raise parser.UnexpectedTokenError(first, "predicate, function or literal")
        sensor.effect = sensor.knowledge_effect()

        return sensor
