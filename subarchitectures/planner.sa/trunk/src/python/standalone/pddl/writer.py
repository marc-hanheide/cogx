#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict

import mapltypes as types
import predicates, conditions, effects
import durative

class Writer(object):
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
            if term.get_type().equal_or_subtype_of(types.t_number):
                if isinstance(term.object.name, float):
                    return "%.4f" % term.object.name
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
        toplevel = [types.t_object, types.t_number]
        for type in _types:
            if type.__class__ != types.Type:
                continue
            if type.supertypes:
                for st in type.supertypes:
                    #only write the lowest supertype(s)
                    if not any(st.is_supertype_of(t) for t in type.supertypes):
                        strings.append("%s - %s" % (type.name, st.name))
            elif type not in toplevel:
                toplevel.append(type)
                
        toplevel.remove(types.t_object)
        toplevel.remove(types.t_number)
        
        strings.append(" ".join(t.name for t in toplevel))
        return self.section(":types", strings)

    def write_objects(self, name, objs):
        strings = []
        per_type = defaultdict(set)
        for o in objs:
            per_type[o.type].add(o)

        for t, olist in per_type.iteritems():
            ostr = " ".join(sorted(o.name for o in olist))
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
        if not strings:
            return []
        return self.section(":predicates", sorted(strings))

    def write_functions(self, funcs):
        strings = []
        for func in funcs:
            if not func.builtin:
                strings.append(self.write_function(func))
        if not strings:
            return []
        return self.section(":functions", sorted(strings))
    
    def write_action(self, action, head=":action"):
        strings = [action.name]
        if action.args:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.args)], parens=False)
        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.effect:
            strings += self.section(":effect", self.write_effect(action.effect), parens=False)

        return self.section(head, strings)

    def write_axiom(self, axiom):
        strings = ["(%s %s)" % (axiom.predicate.name, self.write_typelist(axiom.args))]
        if axiom.condition:
            strings += self.write_condition(axiom.condition)

        return self.section(":derived", strings)
    
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
        if action.args:
            args = [a for a in action.args if a.name != "?duration"]
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(args)], parens=False)
            
        strings += self.section(":duration", self.write_durations(action.duration), parens=False)
        
        if action.precondition:
            strings += self.section(":condition", self.write_condition(action.precondition), parens=False)
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.effect:
            strings += self.section(":effect", self.write_effect(action.effect), parens=False)

        return self.section(":durative-action", strings)


    def write_effect(self, effect):
        if isinstance(effect, durative.TimedEffect):
            return ["(at %s %s)" % (effect.time, self.write_literal(effect))]
        elif isinstance(effect, effects.SimpleEffect):
            return [self.write_literal(effect)]
        elif isinstance(effect, effects.ConjunctiveEffect):
            strings = sum([self.write_effect(e) for e in effect.parts], [])
            return self.section("and", strings)
        elif isinstance(effect, effects.UniversalEffect):
            strings = self.write_effect(effect.effect)
            head  = "forall (%s)" % self.write_typelist(effect.args)
            return self.section(head, strings)
        elif isinstance(effect, effects.ConditionalEffect):
            strings = self.write_effect(effect.effect)
            cond = self.write_condition(effect.condition)
            return self.section("when", cond + strings)
        elif isinstance(effect, effects.ProbabilisticEffect):
            strings = []
            for p,e in effect.effects:
                e_str = self.write_effect(e)
                if p is None:
                    strings += self.section("", e_str, parens=False)
                else:
                    p_str = self.write_term(p)
                    strings += self.section(p_str, e_str, parens=False)
            return self.section("probabilistic", strings)
        else:
            try:
                return effect.write_pddl(self)
            except:
                print "warning: couldn't write:", effect
                return [""]
        
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
            elif cond.__class__ == conditions.PreferenceCondition:
                head = "preference %s" % cond.penalty
            elif cond.__class__ == conditions.Truth:
                head = ""
            elif cond.__class__ == conditions.IntermediateCondition:
                head = "intermediate"
            elif cond.__class__ == durative.TimedCondition:
                if cond.time == "start":
                    head = "at start"
                elif cond.time == "end":
                    head = "at end"
                elif cond.time == "all":
                    head = "over all"
            else:
                return cond.write_pddl(self)
            strings = sum(parts, [])
            return self.section(head, strings)
        
        return condition.visit(write_visitor)
        
    def write_domain(self, domain):
        strings = ["(define (domain %s)" % domain.name]
        strings.append("")
        strings.append("(:requirements %s)" % " ".join(":"+r for r in domain.requirements))
        strings.append("")
        strings += self.write_types(domain.types.itervalues())

        if domain.constants:
            strings.append("")
            strings += self.write_objects("constants", domain.constants)
        
        strings.append("")
        strings += self.write_predicates(domain.predicates)

        strings.append("")
        strings += self.write_functions(domain.functions)

        for a in domain.axioms:
            strings.append("")
            strings += self.write_axiom(a)

        for a in domain.actions:
            strings.append("")
            if isinstance(a, durative.DurativeAction):
                strings += self.write_durative_action(a)
            else:
                strings += self.write_action(a)

        strings.append("")
        strings.append(")")
        return strings

    def write_init(self, inits):
        strings = []
        probstrings = []
        for i in inits:
            if isinstance(i, effects.ProbabilisticEffect):
                probstrings += self.write_effect(i)
            else:
                strings.append(self.write_literal(i))

        return self.section(":init", sorted(strings) + probstrings)
    
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
        if problem.goal:
            strings += self.section(":goal", self.write_condition(problem.goal))

        if problem.optimization:
            strings.append("(:metric %s %s)" % (problem.optimization, self.write_term(problem.opt_func)))
            
        
        strings.append("")
        strings.append(")")
        return strings
            
        
        
