#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
import mapltypes
import scope
import predicates, conditions, actions, domain

from mapltypes import *
from parser import ParseError, UnexpectedTokenError
from actions import Action

class Problem(domain.MAPLDomain):
    def __init__(self, name, objects, init, goal, _domain):
        domain.MAPLDomain.__init__(self, name, _domain.types, _domain.constants, _domain.predicates, _domain.functions, _domain.actions, _domain.sensors, _domain.axioms)
        for o in objects:
            self.add(o)
            
        self.domain = _domain
        self.objects = objects
        self.init = init
        self.goal = goal

    @staticmethod
    def parse(root, domain):
        it = iter(root)
        it.get("define")
        j = iter(it.get(list, "(problem 'problem identifier')"))
        j.get("problem")
        probname = j.get(None, "problem identifier").token.string
        
        j = iter(it.get(list, "domain identifier"))
        j.get(":domain")
        domname = j.get(None, "domain identifier").token

        if domname.string != domain.name:
            raise ParseError(domname, "problem requires domain %s but %s is supplied." % (domname.string, domain.name))

        problem = None
        objects = set()
        
        for elem in it:
            j = iter(elem)
            type = j.get("terminal").token

            
            if type == ":objects":
                olist = mapltypes.parse_typelist(j)
                for key, value in olist.iteritems():
                    if value.string not in domain.types:
                        raise ParseError(value, "undeclared type")

                    objects.add(TypedObject(key.string, domain.types[value.string]))

                problem = Problem(probname, objects, [], None, domain)

            elif type == ":init":
                for elem in j:
                    if elem.isTerminal():
                        raise UnexpectedTokenError(elem.token, "literal or fluent assignment")
                    
                    domain.predicates.add(predicates.equalAssign)
                    domain.predicates.remove(predicates.equals)
                    literal = predicates.Literal.parse(iter(elem), problem, assignOperators=[predicates.equals], maxNesting=0)
                    domain.predicates.remove(predicates.equalAssign)
                    domain.predicates.add(predicates.equals)
                    
                    problem.init.append(literal)

            elif type == ":goal":
                cond = j.get(list, "goal condition")
                problem.goal = conditions.Condition.parse(iter(cond),problem)

            else:
                raise ParseError(type, "Unknown section identifier: %s" % type.string)

        return problem
