import itertools

import predicates, conditions, effects, actions, domain, problem, writer, translators
import mapltypes as types
import builtin
import durative

from parser import ParseError, UnexpectedTokenError
from mapltypes import Type, TypedObject, Parameter
from predicates import Predicate, Function
from builtin import t_object, t_boolean

#basic mapl types
t_agent = Type("agent")
t_planning_agent = Type("planning_agent", [t_agent])
t_subgoal = Type("subgoal")
t_feature = Type("feature")

mapl_types = [t_agent, t_planning_agent, t_subgoal, t_feature]

#mapl predicates
knowledge = Predicate("kval", [Parameter("?a", t_agent), Parameter("?f", types.FunctionType(t_object))], builtin=True)
direct_knowledge = Predicate("kd", [Parameter("?a", t_agent), Parameter("?f", types.FunctionType(t_object))], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
indomain = Predicate("in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
#i_indomain = Predicate("i_in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
not_indomain = Predicate("not_in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

shared_knowledge = Predicate("shval", [Parameter("?a", t_agent), Parameter("?a2", t_agent), Parameter("?f", types.FunctionType(t_object))], builtin=True)

modal_predicates = [knowledge, shared_knowledge, indomain, direct_knowledge, not_indomain]

is_planning_agent = Predicate("is_planning_agent", [Parameter("?a", t_agent)], builtin=True)
achieved = Predicate("achieved", [Parameter("?sg", t_subgoal)], builtin=True)
commited_to_plan = Predicate("commited_to_plan", [Parameter("?a", t_agent)], builtin=True)
can_talk_to = Predicate("can_talk_to", [Parameter("?a1", t_agent), Parameter("?a2", t_agent)], builtin=True)

nonmodal_predicates = [is_planning_agent, achieved, commited_to_plan, can_talk_to]
mapl_predicates = modal_predicates + nonmodal_predicates

#MAPL axioms

kval_axiom = """
(:derived (kval ?a - agent ?svar - (function object))
          (or (kd ?a ?svar)
              (exists (?val - (typeof ?svar)) (= ?svar ?val))
          )
)
"""

in_domain_axiom = """
(:derived (in-domain ?svar - (function object) ?val - (typeof ?svar))
          (or (= ?svar ?val)
              (and (not (not_in-domain ?svar ?val))
                   (not (exists (?val2 - (typeof ?svar)) (= ?svar ?val2))))
          )
)
"""

mapl_axioms = [kval_axiom, in_domain_axiom]


class SenseEffect(object):
    def __init__(self, sense, sensor):
        self.sensor = sensor
        self.sense = sense

    def knowledge_effect(self):
        term = self.get_term()
        if not term:
            return None
        return effects.SimpleEffect(direct_knowledge, [predicates.VariableTerm(self.sensor.agents[0]), term])

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

    def copy(self, newsensor=None):
        if not newsensor:
            newsensor = self.sensor

        if isinstance(self.sense, predicates.Literal):
            s2 = self.sense.copy(newsensor)
        else:
            s2 = predicates.FunctionTerm(self.sense.function, newsensor.lookup(self.sense.args))
        return SenseEffect(s2, newsensor)

    def __eq__(self, other):
        return self.sense == other.sense

    def __neq__(self, other):
        return self.sense != other.sense

    @staticmethod
    def parse(it, scope):
        first = it.get("terminal", "predicate or function").token
        if first.string in scope.predicates:
            return SenseEffect(predicates.Literal.parse(it.reset(), scope), scope)
        elif first.string in scope.functions:
            term = predicates.FunctionTerm.parse(it.reset(), scope)
            return SenseEffect(term, scope)
        else:
            raise parser.UnexpectedTokenError(first, "predicate, function or literal")
    
class MAPLAction(actions.Action):
    def __init__(self, name, agents, args, vars, precondition, replan, effect, sensors, domain):
        actions.Action.__init__(self, name, agents+args+vars, precondition, effect, domain, replan=replan)
        self.agents = agents
        self.maplargs = args
        self.vars = vars
        self.sensors = sensors

    def to_pddl(self):
        str = ["(:action %s" % self.name]
        indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        agents = [Parameter(p.name, p.type) for p in self.agents]
        args = [Parameter(p.name, p.type) for p in self.maplargs]
        vars = [Parameter(p.name, p.type) for p in self.vars]
        
        a = MAPLAction(self.name, agents, args, vars, None, None, None, [], newdomain)

        for arg in a.args:
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(a[arg.type.parameter])
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        if self.effect:
            a.effect = self.effect.copy(a)
        a.sensors = [s.copy(a) for s in self.sensors]

        return a
    
    def knowledge_effect(self):
        effs = [s.knowledge_effect() for s in self.sensors]
        return effects.ConjunctiveEffect(effs)

    def is_pure_sensor(self):
        return not self.effect and self.sensors
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string

        it.get(":agent")
        agent = predicates.parse_arg_list(iter(it.get(list, "agent parameter")), scope.types)

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types, previous_params=agent)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parse_arg_list(iter(it.get(list, "variables")), scope.types, previous_params=agent+params)
            next = it.get()
        else:
            variables = []

        action = MAPLAction(name, agent, params, variables, None, None, None, [], scope)
        
        try:
            while True:
                if next.token.string == ":precondition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action)
                elif next.token.string == ":sense":
                    action.sensors.append(SenseEffect.parse(iter(it.get(list, "sensor specification")), action))
                else:
                    raise UnexpectedTokenError(next.token)
                    
                next = it.next()

        except StopIteration, e:
            pass
        
        return action

class MAPLDurativeAction(MAPLAction, durative.DurativeAction):
    def __init__(self, name, agents, args, vars, duration, precondition, replan, effect, sensors, domain):
        MAPLAction.__init__(self, name, agents, args, vars, precondition, replan, effect, sensors, domain)
        self.add(TypedObject("?duration", types.t_number))
        self.duration = duration
        for d in self.duration:
            d.set_scope(self)

    def copy(self, newdomain=None):
        a = MAPLAction.copy(self, newdomain)
        a.__class__ = MAPLDurativeAction
        a.duration = [durative.DurationConstraint(a.lookup([d.term])[0], d.timeSpecifier) for d in self.duration]
        return a
       
    @staticmethod
    def parse(it, scope):
        it.get(":durative-action")
        name = it.get().token.string

        it.get(":agent")
        agent = predicates.parse_arg_list(iter(it.get(list, "agent parameter")), scope.types)

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types, previous_params=agent)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parse_arg_list(iter(it.get(list, "variables")), scope.types, previous_params=agent+params)
            next = it.get()
        else:
            variables = []
            
        action =  MAPLDurativeAction(name, agent, params, variables, [], None, None, None, [], scope)
        
        next.token.check_keyword(":duration")
        action.duration = durative.DurationConstraint.parse(iter(it.get(list, "duration constraint")), action)

        try:
            while True:
                next = it.next()
                
                if next.token.string == ":condition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = durative.TimedCondition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action, timed_effects=True)
                elif next.token.string == ":sense":
                    action.sensors.append(SenseEffect.parse(iter(it.get(list, "sensor specification")), action))
                else:
                    raise UnexpectedTokenError(next.token)

        except StopIteration, e:
            pass
        
        return action

    
class MAPLWriter(writer.Writer):
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

    def write_action(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.maplargs:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.maplargs)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)
            
        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.effect:
            strings += self.section(":effect", self.write_effect(action.effect), parens=False)

        for se in action.sensors:
            strings += self.write_sense_effect(se)
            
        return self.section(":action", strings)
        
    def write_durative_action(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.maplargs:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.maplargs)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)
            
        strings += self.section(":duration", self.write_durations(action.duration), parens=False)
        
        if action.precondition:
            strings += self.section(":condition", self.write_condition(action.precondition), parens=False)
        if action.replan:
            strings += self.section(":replan", self.write_condition(action.replan), parens=False)
        if action.effect:
            strings += self.section(":effect", self.write_effect(action.effect), parens=False)

        for se in action.sensors:
            strings += self.write_sense_effect(se)

        return self.section(":durative-action", strings)

    
    def write_sense_effect(self, sensor):
        eff = None
        if isinstance(sensor.sense, predicates.Literal):
            eff = self.write_literal(sensor.sense)
        elif isinstance(sensor.sense, predicates.FunctionTerm):
            eff = self.write_term(sensor.sense)
        return self.section(":sense", [eff], parens=False)
        
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
            const = [c for c in domain.constants if c not in (types.TRUE, types.FALSE, types.UNKNOWN)]
            strings += self.write_objects("constants", const)
        for a in domain.actions:
            strings.append("")
            if isinstance(a, MAPLDurativeAction):
                strings += self.write_durative_action(a)
            else:
                strings += self.write_action(a)

        strings.append("")
        strings.append(")")
        return strings
           
class MAPLObjectFluentNormalizer(translators.ObjectFluentNormalizer):
    def translate_action(self, action, domain=None):
        assert domain is not None

        termdict = {}
        pre = self.translate_condition(action.precondition, termdict, domain)
        replan = self.translate_condition(action.replan, termdict, domain)
        effect = self.translate_effect(action.effect, termdict, domain)

        sensors = []
        for se in action.sensors:
            if isinstance(se.get_value(), predicates.FunctionTerm):
                if se.get_value() in termdict:
                    param = termdict[se.get_value()]
                else:
                    param = self.create_param("?val", se.get_term().function.type, set(p.name for p in termdict.itervalues()))
                    termdict[se.get_value()] = param
                se2 = SenseEffect(predicates.Literal(se.sense.predicate, [se.get_term(), predicates.Term(param)]), action)
                sensors.append(se2)
            else:
                sensors.append(se.copy(action))
        
        agents = [types.Parameter(p.name, p.type) for p in action.agents]
        args = [types.Parameter(p.name, p.type) for p in action.maplargs]
        vars = [types.Parameter(p.name, p.type) for p in action.vars]
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(builtin.equals, [term, predicates.Term(param)]))
                vars.append(param)

        return MAPLAction(action.name, agents, args, vars, pre, replan, effect, sensors, domain)

    # def translate_sensor(self, sensor, domain=None):
    #     import sensors
    #     assert domain is not None

    #     termdict = {}
    #     pre = self.translate_condition(sensor.precondition, termdict, domain)

                

    #     agents = [types.Parameter(p.name, p.type) for p in sensor.agents]
    #     args = [types.Parameter(p.name, p.type) for p in sensor.maplargs]
    #     vars = [types.Parameter(p.name, p.type) for p in sensor.vars]
    #     if termdict:
    #         if pre and not isinstance(pre, conditions.Conjunction):
    #             pre = conditions.Conjunction([pre])
    #         elif not pre:
    #             pre = conditions.Conjunction([])
    #         for term, param in termdict.iteritems():
    #             pre.parts.append(conditions.LiteralCondition(builtin.equals, [term, predicates.Term(param)]))
    #             vars.append(param)

    #     return sensors.Sensor(sensor.name, agents, args, vars, pre, senses, domain)
    

