import parser, predicates, conditions, effects, actions, writer, translators
import mapltypes as types
import builtin
import durative

from scope import Scope, SCOPE_CONDITION
from parser import ParseError, UnexpectedTokenError
from mapltypes import Type, Parameter
from predicates import Predicate
from builtin import t_object, t_any

pddl_module = True

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
i_indomain = Predicate("i_in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
attributed = Predicate("attributed", [Parameter("?a", t_agent), p,Parameter("?v",types.ProxyType(p)),], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
neg_attributed = Predicate("neg-attributed", [Parameter("?a", t_agent), p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
#not_indomain = Predicate("not_in-domain", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

p = Parameter("?f", types.FunctionType(t_object))
hyp = Predicate("hyp", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
commit = Predicate("poss", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

p = Parameter("?f", types.FunctionType(t_object))
committed = Predicate("committed", [p], builtin=True)

p = Parameter("?f", types.FunctionType(t_object))
update = Predicate("update", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)
p = Parameter("?f", types.FunctionType(t_object))
update_fail = Predicate("update-failed", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

defined = Predicate("defined", [Parameter("?f", types.FunctionType(t_any))], builtin=True)

failure_cost = predicates.Function("failure-cost", [], builtin.t_number, builtin=True)

# shared_knowledge = Predicate("shval", [Parameter("?a", t_agent), Parameter("?a2", t_agent), Parameter("?f", types.FunctionType(t_object))], builtin=True)

modal_predicates = [knowledge, indomain,\
                    direct_knowledge, i_indomain, \
                    commit, committed, attributed, neg_attributed, \
                    update, update_fail, defined]
# shared_knowledge, , ,\

is_planning_agent = Predicate("is_planning_agent", [Parameter("?a", t_agent)], builtin=True)
achieved = Predicate("achieved", [Parameter("?sg", t_subgoal)], builtin=True)
commited_to_plan = Predicate("commited_to_plan", [Parameter("?a", t_agent)], builtin=True)
can_talk_to = Predicate("can_talk_to", [Parameter("?a1", t_agent), Parameter("?a2", t_agent)], builtin=True)

nonmodal_predicates = [is_planning_agent, achieved, commited_to_plan, can_talk_to]
mapl_predicates = modal_predicates + nonmodal_predicates

default_types = mapl_types
default_constants = [builtin.TRUE, builtin.FALSE, builtin.UNKNOWN]
default_predicates = mapl_predicates
default_functions = [failure_cost]

default_compiler = translators.MAPLCompiler
dependencies = ['modal-predicates']

#MAPL axioms

kval_axiom = """
(:derived (kval ?a - agent ?svar - (function object))
          (or (kd ?a ?svar)
              (exists (?val - (typeof ?svar)) (= ?svar ?val))
          )
)
"""

# hyp_axiom = """
# (:derived (hyp ?svar - (function object) ?val - (typeof ?svar))
#           (or (= ?svar ?val)
#               (and (commit ?svar ?val)
#                    ;;(i_in-domain ?svar ?val)
#                    )
#           )
# )
# """

committed_axiom = """
(:derived (committed ?svar - (function object))
          (exists (?val - (typeof ?svar)) (or (= ?svar ?val)
                                              (poss ?svar ?val)))
)
"""

in_domain_axiom = """
(:derived (in-domain ?svar - (function object) ?val - (typeof ?svar))
          (or (= ?svar ?val)
              (and (i_in-domain ?svar ?val)
                   (not (exists (?val2 - (typeof ?svar)) (= ?svar ?val2))))
          )
)
"""

mapl_axioms = [kval_axiom, in_domain_axiom, committed_axiom]

def prepare_domain(domain):
    domain.init_rules = []

def action_handler(it, domain):
    domain.actions.append(MAPLAction.parse(it, domain))
    return True

def durative_handler(it, domain):
    if "durative-actions" not in domain.requirements:
        return False

    action = MAPLDurativeAction.parse(it, domain)
    action.set_tag("durative_action", True)
    domain.actions.append(action)
    return True

def initrule_handler(it, domain):
    domain.init_rules.append(InitRule.parse(it, domain))
    return True

parse_handlers = {
    ":action" : action_handler,
    ":durative-action" : durative_handler,
    ":init-rule" : initrule_handler
    }

def copy_hook(self, result):
    result.init_rules = [r.copy(result) for r in self.init_rules]

def add_hook(self, result, action):
    if isinstance(action, MAPLAction):
        self.actions.append(action)
        self.name2action = None
    if isinstance(action, InitRule):
        self.init_rules.append(action)

def clear_hook(self, result):
    self.init_rules = []

def get_action_hook(self, result):
    return result + self.init_rules

domain_hooks = {
    'copy' : copy_hook,
    'add_action' : add_hook,
    'clear_actions' : clear_hook,
    'get_action_like' : get_action_hook
    }

        
def post_parse(domain):
    import axioms
    for axiom_str in mapl_axioms:
        axiom = parser.Parser.parse_as(axiom_str.split("\n"), axioms.Axiom, domain)
        domain.axioms.append(axiom)
    

class SenseEffect(object):
    def __init__(self, sense, sensor):
        self.sensor = sensor
        self.sense = sense

    def knowledge_effect(self):
        term = self.get_term()
        if not term:
            return None
        return effects.SimpleEffect(direct_knowledge, [predicates.VariableTerm(self.sensor.agents[0]), term])

    def commit_condition(self):
        if not self.is_boolean():
            return None
        term = self.get_term()
        value = self.get_value()
        if not term:
            return None
        return conditions.LiteralCondition(commit, [term, value])

    def conditional_knowledge_effect(self, add_condition=None):
        cc = self.commit_condition()
        if not cc and not add_condition:
            return self.knowledge_effect()
        elif add_condition:
            cc = conditions.Conjunction.new(cc)
            cc.parts.append(add_condition)
        return effects.ConditionalEffect(cc, self.knowledge_effect())
    
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

    def get_scope(self):
        return self.sensor
    
    def set_scope(self, new_scope):
        """Set a new scope for this Effect and all its children

        Arguments:
        new_scope -- Scope object"""
        assert new_scope is None or isinstance(new_scope, Scope)
        self.sensor = new_scope

        if isinstance(self.sense, predicates.Literal):
            self.sense.set_scope(self.sensor)
        elif self.sensor is not None:
            self.sense = predicates.FunctionTerm(self.sense.function, self.sensor.lookup(self.sense.args))
        
    def __eq__(self, other):
        return self.sense == other.sense

    def __neq__(self, other):
        return self.sense != other.sense

    @staticmethod
    def parse(it, scope):
        first = it.get("terminal", "predicate or function").token
        if first.string in scope.predicates:
            return SenseEffect(predicates.Literal.parse(it.reset(), scope, function_scope=SCOPE_CONDITION), scope)
        elif first.string in scope.functions:
            term = predicates.FunctionTerm.parse(it.reset(), scope)
            return SenseEffect(term, scope)
        else:
            raise parser.UnexpectedTokenError(first, "predicate, function or literal")
    
class MAPLAction(actions.Action):
    def __init__(self, name, agents, params, vars, precondition, replan, effect, sensors, domain):
        actions.Action.__init__(self, name, agents+params+vars, precondition, effect, domain, replan=replan)
        self._agents = [a.name for a in agents]
        self._params = [a.name for a in params]
        self._vars = [a.name for a in vars]
        self.sensors = sensors

    def _get_args(self, names):
        adict = dict((a.name, a) for a in self.args)
        return [adict[n] for n in names]
    def _set_args(self, names, newvals):
        names[:] = [a.name for a in newvals]

    agents = property(lambda self: self._get_args(self._agents), lambda self, x: self._set_args(self._agents, x))
    params = property(lambda self: self._get_args(self._params), lambda self, x: self._set_args(self._params, x))
    vars = property(lambda self: self._get_args(self._vars), lambda self, x: self._set_args(self._vars, x))

    # def to_pddl(self):
    #     str = ["(:action %s" % self.name]
    #     indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent

        args = [Parameter(p.name, p.type) for p in self.args]
        adict = dict((a.name, a) for a in args)
        agents = [adict[a.name] for a in self.agents]
        vars = [adict[a.name] for a in self.vars]
        params = [a for a in args if a not in agents and a not in vars]

        a = MAPLAction(self.name, agents, params, vars, None, None, None, [], newdomain)
        a.args = a.copy_args(self.args)
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        if self.effect:
            a.effect = self.effect.copy(a)
        a.sensors = [s.copy(a) for s in self.sensors]

        return a

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        args = [Parameter(p.name, p.type) for p in self.args]
        adict = dict((a.name, a) for a in args)
        agents = [adict[a.name] for a in self.agents]
        vars = [adict[a.name] for a in self.vars]
        params = [a for a in args if a not in agents and a not in vars]
        
        return MAPLAction(self.name, agents, params, vars, None, None, None, [], newdomain)
    
    def knowledge_effect(self):
        effs = [s.knowledge_effect() for s in self.sensors]
        return effects.ConjunctiveEffect(list(set(effs)))

    def commit_condition(self):
        conds = filter(None, [s.commit_condition() for s in self.sensors])
        return conditions.Conjunction(list(set(conds)))

    def conditional_knowledge_effect(self):
        effs = [s.conditional_knowledge_effect() for s in self.sensors]
        return effects.ConjunctiveEffect(list(set(effs)))
    
    def is_pure_sensor(self):
        return not self.effect and self.sensors
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string

        try:
            it.get(":agent")
        except parser.UnexpectedTokenError:
            ## not a mapl action?
            return actions.Action.parse(it.reset(), scope)
            
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

        except StopIteration:
            pass
        
        return action

class MAPLDurativeAction(MAPLAction, durative.DurativeAction):
    def __init__(self, name, agents, params, vars, duration, precondition, replan, effect, sensors, domain):
        d = Parameter("?duration", types.t_number)
        MAPLAction.__init__(self, name, agents, params, vars + [d], precondition, replan, effect, sensors, domain)

        self.set_tag("durative_action", True) # proper parsing context
        
        self.duration = duration
        for d in self.duration:
            d.set_scope(self)

    def instantiate(self, mapping, parent=None):
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        mapping["?duration"] = self.duration[0].term.copy_instance()
        actions.Action.instantiate(self, mapping, parent)

    def knowledge_effect(self):
        effs = [s.knowledge_effect() for s in self.sensors]
        t_effs = [durative.TimedEffect(e.predicate, e.args, "end", e.get_scope(), e.negated) for e in effs]
        return effects.ConjunctiveEffect(t_effs)

    def commit_condition(self):
        conds = filter(None, [s.commit_condition() for s in self.sensors])
        return durative.TimedCondition("start", conditions.Conjunction(conds))
    
    def copy(self, newdomain=None):
        a = MAPLAction.copy(self, newdomain)
        a.__class__ = MAPLDurativeAction
        a.duration = [durative.DurationConstraint(a.lookup([d.term])[0], d.timeSpecifier) for d in self.duration]
        return a

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""

        a = MAPLAction.copy_skeleton(self, newdomain)
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
                    action.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.set_tag("durative_action", False)
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                    action.set_tag("durative_action", True)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action)
                elif next.token.string == ":sense":
                    action.sensors.append(SenseEffect.parse(iter(it.get(list, "sensor specification")), action))
                else:
                    raise UnexpectedTokenError(next.token)

        except StopIteration:
            pass
        
        return action


class InitRule(actions.Action):
    def __init__(self, name, args, precondition, effect, domain):
        actions.Action.__init__(self, name, args, precondition, effect, domain)

    def get_effects(self):
        if not self.effect:
            return []
        if isinstance(self.effect, effects.ConjunctiveEffect):
            return self.effect.parts
        return [self.effect]

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent

        r = InitRule(self.name, [], None, None, newdomain)
        r.args = r.copy_args(self.args)
        
        if self.precondition:
            r.precondition = self.precondition.copy(r)
        if self.effect:
            r.effect = self.effect.copy(r)

        return r

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        r = InitRule(self.name, [], None, None, newdomain)
        r.args = r.copy_args(self.args)
        return r
    
    
    @staticmethod
    def parse(it, scope):
        it.get(":init-rule")
        name = it.get().token.string
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        rule = InitRule(name, params, None, None, scope)

        try:
            while True:
                if next.token.string == ":precondition":
                    if rule.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    rule.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), rule)
                elif next.token.string == ":effect":
                    if rule.effect:
                        raise ParseError(next.token, "effects already defined.")
                    rule.effect = effects.Effect.parse(iter(it.get(list, "effect")), rule)
                else:
                    raise UnexpectedTokenError(next.token)
                    
                next = it.next()

        except StopIteration:
            pass
            
        return rule

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
        if not isinstance(action, MAPLAction):
            return writer.Writer.write_action(self, action)
        
        strings = [action.name]
        params = [a for a in action.args if a not in action.agents and a not in action.vars]

        if action.agents:
            strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if params:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(params)], parens=False)
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

    def write_init_rule(self, rule):
        return writer.Writer.write_action(self, rule, head=":init-rule")
    
    def write_durative_action(self, action):
        strings = [action.name]

        params = [a for a in action.args if a not in action.agents and a not in action.vars]
        if action.agents:
            strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if params:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(params)], parens=False)
        vars = [a for a in action.vars if a.name != "?duration"]
        if vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(vars)], parens=False)
            
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

        for r in domain.init_rules:
            strings.append("")
            strings += self.write_init_rule(r)
            
        for a in domain.axioms:
            strings.append("")
            strings += self.write_axiom(a)
            
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
        
        add_args = []
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(builtin.equals, [term, predicates.Term(param)]))
                add_args.append(param)

        a2 = action.copy_skeleton(domain)
        a2.add(add_args)
        a2.vars += add_args
        a2.args += add_args

        if pre:
            pre.set_scope(a2)
            a2.precondition = pre
        if replan:
            replan.set_scope(a2)
            a2.replan = replan
        if effect:
            effect.set_scope(a2)
            a2.effect = effect
        for s in sensors:
            s.set_scope(a2)
            a2.sensors.append(s)
            
        return a2

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
    
