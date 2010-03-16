#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
import mapltypes as types
import predicates, conditions, effects, mapl

aadasjippiguvkl    
class Sensor(mapl.MAPLAction):
    def __init__(self, name, agents, args, vars, precondition, senses, domain):
        mapl.MAPLAction.__init__(self, name, agents, args, vars, precondition, None, None, domain)
        self.senses = senses
        self.effect = self.knowledge_effect()

    def knowledge_effect(self):
        effs = [s.knowledge_effect() for s in self.senses]
        return effects.ConjunctiveEffect(effs)

    def copy(self, newdomain=None):
        a = super(type(self), self).copy(newdomain)
        a.__class__ = self.__class__

        a.senses = [s.copy(a) for s in self.senses]

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

        sensor =  Sensor(name, agent, params, variables, None, [], scope)
        

        if next.token.string == ":precondition":
            sensor.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), sensor)
            next = it.get()
        else:
            precondition = None

        next.token.check_keyword(":sense")
        j = iter(it.get(list, "function term or literal"))
        first = j.get("terminal", "conjunction of predicate or function").token

        if first.string == "and":
            senses = []
            for part in j:
                if part.is_terminal():
                    raise UnexpectedTokenError(part.token, "function term or literal")
                senses.append(iter(part))
        else:
            senses = [j.reset()]

        for s in senses:
            first = s.get("terminal", "predicate or function").token
            if first.string in scope.predicates:
                sensor.senses.append(SenseEffect(predicates.Literal.parse(s.reset(), sensor), sensor))
            elif first.string in scope.functions:
                term = predicates.FunctionTerm.parse(s.reset(), sensor)
                sensor.senses.append(SenseEffect(term, sensor))
            else:
                raise parser.UnexpectedTokenError(first, "predicate, function or literal")
        sensor.effect = sensor.knowledge_effect()

        return sensor
