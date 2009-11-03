#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools

import parser
import mapltypes
import scope
import predicates, conditions, actions, effects, domain

from mapltypes import *
from parser import ParseError, UnexpectedTokenError
from actions import Action
from problem import Problem

def product(*iterables):
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)

class MapsimScenario(object):
    def __init__(self, name, world, agents, domain):
        self.name = name
        self.domain = domain
        self.world = world
        self.agents = agents

    @staticmethod
    def parse(root, domain):
        it = iter(root)
        it.get("define")
        j = iter(it.get(list, "(problem 'scenario identifier')"))
        j.get("scenario")
        scname = j.get(None, "scenario identifier").token.string
        
        j = iter(it.get(list, "domain identifier"))
        j.get(":domain")
        domname = j.get(None, "domain identifier").token

        if domname.string != domain.name:
            raise ParseError(domname, "scenario requires domain %s but %s is supplied." % (domname.string, domain.name))

        scenario = None
        common = None
        world = None
        agents = {}
        
        for elem in it:
            section, problem, agent = MapsimScenario.parseSection(iter(elem), scname, domain, common)
            if section == ":world":
                if world is not None:
                    raise ParseError(section, "Duplicate problem specification for world state.")
                world = problem
            elif section == ":common":
                if common is not None:
                    raise ParseError(section, "Duplicate specification of common state.")
                common = problem
            elif section == ":agent":
                if agent in agents:
                    raise ParseError(section, "Duplicate problem specification for agent '%s'.")
                agents[agent] = problem

        if not world:
            if not common:
                raise ParseError(root.token, "Neither world state nor common state are specified.")
            world = Problem(scname+"-world", common.objects, common.init, common.goal, domain)

        if not agents:
            raise ParseError(root.token, "No agents are defined.")
                
        for agent in agents.iterkeys():
            if agent not in world:
                raise ParseError(root.token, "Agent '%s' is not defined in the world state." % agent)
            
        return MapsimScenario(scname, world, agents, domain)

    @staticmethod
    def parseSection(it, scenario_name, domain, common):
        section = it.get("terminal").token
        agent = None
        if section == ":agent":
            agent = it.get("terminal", "agent name").token
            name = "%s-%s" % (scenario_name, agent.string)
        elif section.string in (":common", ":world"):
            name = "%s-%s" % (scenario_name, section.string[1:])
        else:
            raise UnexpectedTokenError(section, "':common', ':world' or ':agent'")

        if not common:
            problem = Problem(name, [], [], None, domain)
        else:
            problem = Problem(name, common.objects, common.init, common.goal, domain)
        
        for elem in it:
            j = iter(elem)
            type = j.get("terminal").token
            
            if type == ":objects":
                olist = mapltypes.parse_typelist(j)
                for key, value in olist.iteritems():
                    if value.string not in domain.types:
                        raise ParseError(value, "undeclared type")

                    problem.addObject(TypedObject(key.string, domain.types[value.string]))

            elif type == ":init":
                for elem in j:
                    if elem.isTerminal():
                        raise UnexpectedTokenError(elem.token, "literal or fluent assignment")
                        
                    init_elem = Problem.parseInitElement(iter(elem), problem)
                    problem.init.append(init_elem)

            elif type == ":goal":
                cond = j.get(list, "goal condition")
                problem.goal = conditions.Condition.parse(iter(cond),problem)

            elif type == ":metric":
                opt = j.get("terminal", "optimization").token
                if opt.string not in ("minimize", "maximize"):
                    raise UnexpectedTokenError(opt, "'minimize' or 'maximize'")
                problem.optimization = opt.string
                
                problem.functions.add(predicates.total_time)
                func = predicates.Term.parse(j,problem)
                problem.functions.remove(predicates.total_time)

                j.noMoreTokens()

                if not isinstance(func.getType(), FunctionType):
                    raise ParseError(elem.token, "Optimization function can't be a constant.")
                if not func.getType().equalOrSubtypeOf(numberType):
                    raise ParseError(elem.token, "Optimization function must be numeric.")
                
                problem.opt_func = func

            else:
                raise ParseError(type, "Unknown section identifier: %s" % type.string)

        if agent:
            if agent.string not in problem:
                raise ParseError(agent , "Agent '%s' is not defined in its problem." % agent.string)
            agent = agent.string
        
        return section, problem, agent
    
