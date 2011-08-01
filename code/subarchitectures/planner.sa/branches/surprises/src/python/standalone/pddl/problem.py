#! /usr/bin/env python
# -*- coding: latin-1 -*-
import itertools

import mapltypes as types
import builtin
import predicates, conditions, effects

from scope import Scope, SCOPE_INIT
from parser import ParseError, UnexpectedTokenError

def product(*iterables):
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)

class Problem(Scope):
    def __init__(self, name, objects, init, goal, _domain, optimization=None, opt_func=None):
        assert all(o.is_instance_of(builtin.t_object) for o in objects)
        problem_objects = set(o for o in objects if o not in _domain.constants)
        Scope.__init__(self, problem_objects, _domain)
        self.set_tag("only-simple-effects", True, inherit=False)

        self.name = name

        self.objects = problem_objects
        self.init = [l.copy(self) for l in init]
        self.goal = None
        if goal:
            self.goal = goal.copy(self)
        self.optimization = optimization
        self.opt_func = opt_func
        self.objects_by_type = {}

    @property
    def domain(self):
        return self.parent

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.domain
            
        return self.__class__(self.name, self.objects, self.init, self.goal, newdomain, self.optimization, self.opt_func)

    def add_object(self, object):
        assert object.is_instance_of(builtin.t_object)
        if object.name in self:
            self.objects.remove(self[object.name])
        self.objects.add(object)
        self.add(object)
        for typ, objs in self.objects_by_type.iteritems():
            if object.is_instance_of(typ):
                objs.add(object)

    def remove_object(self, object):
        if dict.__contains__(self, object.name):
            self.objects.remove(self[object.name])
            del self[object.name]
            for typ, objs in self.objects_by_type.iteritems():
                if object.is_instance_of(typ):
                    objs.discard(object)

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
            if isinstance(type, types.ProxyType):
                type = type.effective_type()
            if type not in self.objects_by_type:
                self.objects_by_type[type] = set(o for o in self.objects if o.is_instance_of(type) and o != builtin.UNKNOWN)
            for obj in self.domain.get_all_objects(type):
                yield obj
            for obj in self.objects_by_type[type]:
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

                #problem.functions.add(builtin.total_cost)
                func = predicates.Term.parse(j,problem)
                #problem.functions.remove(builtin.total_cost)

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
            return effects.ProbabilisticEffect.parse(it.reset(), scope)
        elif first.string == "assign-probabilistic":
            return effects.ProbabilisticEffect.parse_assign(it.reset(), scope)
        else:
            lit=  predicates.Literal.parse(it.reset(), scope, maxNesting=0, function_scope=SCOPE_INIT)

            return lit

