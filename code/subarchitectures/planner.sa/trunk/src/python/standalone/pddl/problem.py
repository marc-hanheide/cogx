#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import parser
import mapltypes as types
import builtin
import scope
import predicates, conditions, actions, effects, domain

from parser import ParseError, UnexpectedTokenError
from actions import Action

def product(*iterables):
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)

class Problem(domain.Domain):
    def __init__(self, name, objects, init, goal, _domain, optimization=None, opt_func=None):
        domain.Domain.__init__(self, name, _domain.types, _domain.constants, _domain.predicates, _domain.functions, [], [])
        self.actions = [a.copy(self) for a in _domain.actions]
        self.axioms = [a.copy(self) for a in _domain.axioms]
        if _domain.observe:
            self.observe = [o.copy(self) for o in _domain.observe]
            
        self.stratify_axioms()
        self.name2action = None
        
        for o in objects:
            self.add(o)
            
        self.domain = _domain
        self.requirements = self.domain.requirements

        self.objects = set(o for o in objects)
        self.init = [l.copy(self) for l in init]
        self.goal = None
        if goal:
            self.goal = goal.copy(self)
        self.optimization = optimization
        self.opt_func = opt_func

    def copy(self):
        return self.__class__(self.name, self.objects, self.init, self.goal, self.domain, self.optimization, self.opt_func)

    def add_object(self, object):
        if object.name in self:
            self.objects.remove(self[object.name])
        self.objects.add(object)
        self.add(object)

    def get_all_objects(self, type):
        if isinstance(type, types.FunctionType):
            for func in self.functions:
                if func.builtin:
                    continue
                #print func.name, types.FunctionType(func.type).equal_or_subtype_of(type)
                if types.FunctionType(func.type).equal_or_subtype_of(type):
                    combinations = product(*map(lambda a: list(self.get_all_objects(a.type)), func.args))
                    for c in combinations:
                        #print FunctionTerm(func, c, self.problem)
                        yield predicates.FunctionTerm(func, c, self)
        else:
            for obj in itertools.chain(self.objects, self.constants):
                if obj.is_instance_of(type) and obj != builtin.UNKNOWN:
                    yield obj
        

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
                olist = types.parse_typelist(j)
                for key, value in olist.iteritems():
                    if value.string not in domain.types:
                        raise ParseError(value, "undeclared type")

                    objects.add(types.TypedObject(key.string, domain.types[value.string]))
                problem = Problem(probname, objects, [], None, domain)

            elif type == ":init":
                for elem in j:
                    if elem.is_terminal():
                        raise UnexpectedTokenError(elem.token, "literal or fluent assignment")
                        
                    init_elem = Problem.parseInitElement(iter(elem), problem)
                    problem.init.append(init_elem)
                    
            elif type == ":goal":
                cond = j.get(list, "goal condition")
                problem.goal = conditions.Condition.parse(iter(cond),problem)
                j.no_more_tokens()

            elif type == ":metric":
                opt = j.get("terminal", "optimization").token
                if opt.string not in ("minimize", "maximize"):
                    raise UnexpectedTokenError(opt, "'minimize' or 'maximize'")
                problem.optimization = opt.string
                
                problem.functions.add(builtin.total_time)
                problem.functions.add(builtin.total_cost)
                func = predicates.Term.parse(j,problem)
                problem.functions.remove(builtin.total_time)
                problem.functions.remove(builtin.total_cost)

                j.no_more_tokens()

                if not isinstance(func.get_type(), types.FunctionType):
                    raise ParseError(elem.token, "Optimization function can't be a constant.")
                if not func.get_type().equal_or_subtype_of(builtin.t_number):
                    raise ParseError(elem.token, "Optimization function must be numeric.")
                
                problem.opt_func = func

            else:
                raise ParseError(type, "Unknown section identifier: %s" % type.string)

        return problem

    @staticmethod
    def parseInitElement(it, scope):
        first = it.get("terminal").token
        if first.string == "probabilistic":
            #TODO: disallow nested functions in those effects.
            return effects.ProbabilisticEffect.parse(it.reset(), scope, timed_effects=False, only_simple=True)
        elif first.string == "assign-probabilistic":
            return effects.ProbabilisticEffect.parse_assign(it.reset(), scope)
        else:
            scope.predicates.remove(builtin.equals)
            scope.predicates.add(builtin.equal_assign)
            if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
                scope.predicates.remove(builtin.eq)
                scope.predicates.add(builtin.num_equal_assign)
            try:
                lit=  predicates.Literal.parse(it.reset(), scope, maxNesting=0)
            finally:
                if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
                    scope.predicates.remove(builtin.num_equal_assign)
                    scope.predicates.add(builtin.eq)
                scope.predicates.remove(builtin.equal_assign)
                scope.predicates.add(builtin.equals)

            return lit

