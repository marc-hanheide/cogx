#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict

import mapltypes as types
import predicates, conditions, effects, actions, sensors

class MAPLWriter(object):
    def __init__(self):
        pass
    
    def section(self, name, content, parens=True):
        if not content:
            content = [""]
            
        if parens:
            strings = ["(%s  %s" % (name, content[0])]
            indent = " " * (len(name)+3)
        else:
            strings = ["%s  %s" % (name, content[0])]
            indent = " " * (len(name)+2)
        
        for s in content[1:]:
            strings.append(indent + s)
            
        if parens:
            if len(strings) == 1:
                strings[0] += ")"
            else:
                strings.append(")")
            
        return strings

    def write_type(self, type):
        if isinstance(type, types.CompositeType):
            return "(either %s)" % " ".join(self.write_type(t) for t in type.types)
        if isinstance(type, types.FunctionType):
            return "(function %s)" % self.write_type(type.type)
        if isinstance(type, types.ProxyType):
            return "(typeof %s)" % type.parameter.name
        return type.name
    
    def write_term(self, term):
        if isinstance(term, (predicates.ConstantTerm, predicates.VariableTerm)):
            if term.getType().equalOrSubtypeOf(types.numberType):
                return str(term.object.name)
            return term.object.name

        args = " ".join(self.write_term(t) for t in term.args)
        return "(%s %s)" % (term.function.name, args)
    
    def write_literal(self, literal):
        args = " ".join(self.write_term(t) for t in literal.args)
        if literal.negated:
            return "(not (%s %s))" % (literal.predicate.name, args)
        return "(%s %s)" % (literal.predicate.name, args)
        

    def write_typelist(self, args):
        return " ".join("%s - %s" % (arg.name, self.write_type(arg.type)) for arg in args)

    def write_types(self, _types):
        strings = []
        toplevel = [types.objectType, types.numberType]
        for type in _types:
            if type.__class__ != types.Type:
                continue
            if type.supertypes:
                for st in type.supertypes:
                    #only write the lowest supertype(s)
                    if not any(st.isSupertypeOf(t) for t in type.supertypes):
                        strings.append("%s - %s" % (type.name, st.name))
            elif type not in toplevel:
                toplevel.append(type)
                
        toplevel.remove(types.objectType)
        toplevel.remove(types.numberType)
        
        strings.append(" ".join(t.name for t in toplevel))
        return self.section(":types", strings)

    def write_objects(self, name, objs):
        strings = []
        per_type = defaultdict(set)
        for o in objs:
            per_type[o.type].add(o)

        for t, olist in per_type.iteritems():
            ostr = " ".join(o.name for o in olist)
            strings.append("%s - %s" % (ostr, self.write_type(t)))
            
        return self.section(":%s" % name, strings)

    def write_predicate(self, pred):
        return "(%s %s)" % (pred.name, self.write_typelist(pred.args))

    def write_function(self, func):
        return "(%s %s) - %s" % (func.name, self.write_typelist(func.args), self.write_type(func.type))
    
    def write_predicates(self, preds):
        strings = []
        for pred in preds:
            if not pred.builtin:
                strings.append(self.write_predicate(pred))
        return self.section(":predicates", strings)

    def write_functions(self, funcs):
        strings = []
        for func in funcs:
            if not func.builtin:
                strings.append(self.write_function(func))
        return self.section(":functions", strings)
    
    def write_action(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.args:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.args)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)

        strings += self.section(":effect", self.write_effects(action.effects), parens=False)

        return self.section(":action", strings)

    def write_durations(self, durations):
        strings = []
        for dc in durations:
            dstr = "(= ?duration %s)" % self.write_term(dc.term)
            if dc.timeSpecifier:
                dstr = "(at %s %s)" % (dc.timeSpecifier, dstr)
            strings.append(dstr)
                
        if len(durations) > 1:
            return self.section("and", strings)
        return strings

    
    def write_durative_action(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.args:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.args)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)
            
        strings += self.section(":duration", self.write_durations(action.duration), parens=False)
        
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.precondition:
            strings += self.section(":condition", self.write_condition(action.precondition), parens=False)

        strings += self.section(":effect", self.write_effects(action.effects), parens=False)

        return self.section(":durative-action", strings)

    
    def write_sensor(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.args:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.args)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)
        if action.precondition:
            strings += self.section(":precondition", [self.write_condition(action.precondition)], parens=False)

        if isinstance(action.sense, predicates.Literal):
            eff = [self.write_literal(action.sense)]
        elif isinstance(action.sense, predicates.FunctionTerm):
            eff = [self.write_term(action.sense)]
        strings += self.section(":sense", eff, parens=False)
        
        return self.section(":sensor", strings)
        
    def write_effects(self, effects):
        if len(effects) == 1:
            return self.write_effect(effects[0])
        
        strings = sum([self.write_effect(e) for e in effects], [])
        return self.section("and", strings)

    def write_effect(self, effect):
        if isinstance(effect, effects.TimedEffect):
            return ["(at %s %s)" % (effect.time, self.write_literal(effect))]
        elif isinstance(effect, effects.SimpleEffect):
            return [self.write_literal(effect)]
        elif isinstance(effect, effects.UniversalEffect):
            strings = self.write_effects(effect.effects)
            head  = "forall (%s)" % self.write_typelist(effect.args)
            return self.section(head, strings)
        elif isinstance(effect, effects.ConditionalEffect):
            strings = self.write_effects(effect.effects)
            cond = self.write_condition(effect.condition)
            return self.section("when", cond + strings)

        assert False, effect
            
        
    def write_condition(self, condition):
        def write_visitor(cond, parts):
            if cond.__class__ == conditions.LiteralCondition:
                return [self.write_literal(cond)]
            
            if cond.__class__ == conditions.Conjunction:
                head = "and"
            elif cond.__class__ == conditions.Disjunction:
                head = "or"
            elif cond.__class__ == conditions.UniversalCondition:
                head = "forall (%s)" % self.write_typelist(cond.values())
            elif cond.__class__ == conditions.ExistentialCondition:
                head = "exists (%s)" % self.write_typelist(cond.values())
            elif cond.__class__ == conditions.TimedCondition:
                if cond.time == "start":
                    head = "at start"
                elif cond.time == "end":
                    head = "at end"
                elif cond.time == "all":
                    head = "over all"

            strings = sum(parts, [])
            return self.section(head, strings)
        
        return condition.visit(write_visitor)
        
    def write_domain(self, domain):
        strings = ["(define (domain %s)" % domain.name]
        strings.append("")
        strings.append("(:requirements %s)" % " ".join(":"+r for r in domain.requirements))
        strings.append("")
        strings += self.write_types(domain.types.itervalues())

        strings.append("")
        strings += self.write_predicates(domain.predicates)

        strings.append("")
        strings += self.write_functions(domain.functions)
        
        if domain.constants:
            strings.append("")
            strings += self.write_objects("constants", domain.constants)
        for s in domain.sensors:
            strings.append("")
            strings += self.write_sensor(s)
        for a in domain.actions:
            strings.append("")
            if isinstance(a, actions.DurativeAction):
                strings += self.write_durative_action(a)
            else:
                strings += self.write_action(a)

        strings.append("")
        strings.append(")")
        return strings

    def write_init(self, inits):
        strings = []
        for i in inits:
            strings.append(self.write_literal(i))

        return self.section(":init", strings)
    
    def write_problem(self, problem):
        strings = ["(define (problem %s)" % problem.name]
        strings.append("")
        strings.append("(:domain %s)" % problem.domain.name)
        
        if problem.objects:
            strings.append("")
            strings += self.write_objects("objects", problem.objects)

        strings.append("")
        strings += self.write_init(problem.init)
        strings.append("")
        strings += self.section(":goal", self.write_condition(problem.goal))

        if problem.optimization:
            strings.append("(:metric %s %s)" % (problem.optimization, self.write_term(problem.opt_func)))
            
        
        strings.append("")
        strings.append(")")
        return strings
            
        
        
