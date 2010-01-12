import itertools

import predicates, conditions, effects, actions, domain, problem, writer, translators
import mapltypes as types
import durative

class MAPLDomain(domain.Domain):
    def __init__(self, name, types, constants, predicates, functions, actions, sensors, axioms):
        domain.Domain.__init__(self, name, types, constants, predicates, functions, actions, axioms)
        self.sensors = sensors

    def copy(self):
        dom = domain.Domain.copy(self)
        dom.sensors = [s.copy(self) for s in self.sensors]
        return dom

    def getAction(self, name):
        if not self.name2action:
            self.name2action = dict((a.name, a) for a in itertools.chain(self.actions, self.sensors))
        return self.name2action[name]

class MAPLProblem(problem.Problem, MAPLDomain):
    def __init__(self, name, objects, init, goal, _domain, optimization=None, opt_func=None):
        MAPLDomain.__init__(self, name, _domain.types, _domain.constants, _domain.predicates, _domain.functions, [], [], [])
        problem.Problem.__init__(self, name, objects, init, goal, _domain, optimization, opt_func)
        
        self.sensors = [s.copy(self) for s in _domain.sensors]
        self.name2action = None


class MAPLAction(actions.Action):
    def __init__(self, name, agents, args, vars, precondition, replan, effects, domain):
        actions.Action.__init__(self, name, agents+args+vars, precondition, effects, domain, replan=replan)
        self.agents = agents
        self.maplargs = args
        self.vars = vars

    def to_pddl(self):
        str = ["(:action %s" % self.name]
        indent = len("(:action ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        agents = [types.Parameter(p.name, p.type) for p in self.agents]
        args = [types.Parameter(p.name, p.type) for p in self.maplargs]
        vars = [types.Parameter(p.name, p.type) for p in self.vars]
        
        a = MAPLAction(self.name, agents, args, vars, None, None, [], newdomain)

        for arg in a.args:
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(a[arg.type.parameter])
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        a.effects = [e.copy(a) for e in self.effects]

        return a
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string

        it.get(":agent")
        agent = predicates.parseArgList(iter(it.get(list, "agent parameter")), scope.types)

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parseArgList(iter(it.get(list, "variables")), scope.types)
            next = it.get()
        else:
            variables = []

        action = MAPLAction(name, agent, params, variables, None, None, None, scope)
        
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
                    if action.effects:
                        raise ParseError(next.token, "effects already defined.")
                    action.effects = effects.Effect.parse(iter(it.get(list, "effect")), action)
                    
                next = it.next()

        except StopIteration, e:
            pass
        
        return action

class MAPLDurativeAction(MAPLAction, durative.DurativeAction):
    def __init__(self, name, agents, args, vars, duration, precondition, replan, effects, domain):
        MAPLAction.__init__(self, name, agents, args, vars, precondition, replan, effects, domain)
        self.add(types.TypedObject("?duration", types.numberType))
        self.duration = duration

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
        agent = predicates.parseArgList(iter(it.get(list, "agent parameter")), scope.types)

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parseArgList(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        if next.token.string == ":variables":
            variables = predicates.parseArgList(iter(it.get(list, "variables")), scope.types)
            next = it.get()
        else:
            variables = []
            
        action =  MAPLDurativeAction(name, agent, params, variables, None, None, None, [], scope)
        
        next.token.checkKeyword(":duration")
        action.duration = durative.DurationConstraint.parse(iter(it.get(list, "duration constraint")), action)

        next = it.get()
        try:
            while True:
                if next.token.string == ":precondition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = TimedCondition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effects:
                        raise ParseError(next.token, "effects already defined.")
                    action.effects = effects.Effect.parse(iter(it.get(list, "effect")), action, timedEffects=True)
                    
                next = it.next()

        except StopIteration, e:
            pass
        
        return action

    
class MAPLWriter(writer.Writer):
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

    def write_mapl_action(self, action):
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

        strings += self.section(":effect", self.write_effects(action.effects), parens=False)

        return self.section(":action", strings)
        
    def write_durative_mapl_action(self, action):
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

        strings += self.section(":effect", self.write_effects(action.effects), parens=False)

        return self.section(":durative-action", strings)

    
    def write_sensor(self, action):
        strings = [action.name]
        strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
        if action.maplargs:
            strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.maplargs)], parens=False)
        if action.vars:
            strings += self.section(":variables", ["(%s)" % self.write_typelist(action.vars)], parens=False)

        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)

        if isinstance(action.sense, predicates.Literal):
            eff = [self.write_literal(action.sense)]
        elif isinstance(action.sense, predicates.FunctionTerm):
            eff = [self.write_term(action.sense)]
        strings += self.section(":sense", eff, parens=False)
        
        return self.section(":sensor", strings)
        
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
        for s in domain.sensors:
            strings.append("")
            strings += self.write_sensor(s)
        for a in domain.actions:
            strings.append("")
            if isinstance(a, MAPLDurativeAction):
                strings += self.write_durative_mapl_action(a)
            else:
                strings += self.write_mapl_action(a)

        strings.append("")
        strings.append(")")
        return strings
       

class MAPLTranslator(translators.Translator):
    def translate_sensor(self, sensor, domain=None):
        return sensor.copy(newdomain=domain)
    
    def translate_domain(self, domain):
        dom = MAPLDomain(domain.name, domain.types.copy(), domain.constants.copy(), domain.predicates.copy(), domain.functions.copy(), [], [], [])
        dom.requirements = domain.requirements.copy()
        dom.actions = [self.translate_action(a, dom) for a in domain.actions]
        dom.sensors = [self.translate_sensor(s, dom) for s in domain.sensors]
        dom.axioms = [self.translate_axiom(a, dom) for a in domain.axioms]
        dom.stratifyAxioms()
        dom.name2action = None
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        return MAPLProblem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
    
class MAPLObjectFluentNormalizer(translators.ObjectFluentNormalizer, MAPLTranslator):
    def translate_action(self, action, domain=None):
        assert domain is not None

        termdict = {}
        pre = None
        replan = None
        effects = []
        if action.precondition:
            pre = self.translate_condition(action.precondition, termdict, domain)
        if action.replan:
            replan = self.translate_condition(action.replan, termdict, domain)
        for e in action.effects:
            effects.append(self.translate_effect(e, termdict, domain))

        agents = [types.Parameter(p.name, p.type) for p in action.agents]
        args = [types.Parameter(p.name, p.type) for p in action.maplargs]
        vars = [types.Parameter(p.name, p.type) for p in action.vars]
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(predicates.equals, [term, predicates.Term(param)]))
                vars.append(param)

        return MAPLAction(action.name, agents, args, vars, pre, replan, effects, domain)

    def translate_sensor(self, sensor, domain=None):
        import sensors
        assert domain is not None

        termdict = {}
        pre = None
        if sensor.precondition:
            pre = self.translate_condition(sensor.precondition, termdict, domain)

        sense = sensor.sense
        if isinstance(sensor.get_value(), predicates.FunctionTerm):
            if sensor.get_value() in termdict:
                param = termdict[sensor.get_value()]
            else:
                param = self.create_param("?val", sensor.get_term().function.type, set(p.name for p in termdict.itervalues()))
                termdict[sensor.get_value()] = param
            sense = predicates.Literal(sensor.sense.predicate, [sensor.get_term(), predicates.Term(param)])

        agents = [types.Parameter(p.name, p.type) for p in sensor.agents]
        args = [types.Parameter(p.name, p.type) for p in sensor.maplargs]
        vars = [types.Parameter(p.name, p.type) for p in sensor.vars]
        if termdict:
            if pre and not isinstance(pre, conditions.Conjunction):
                pre = conditions.Conjunction([pre])
            elif not pre:
                pre = conditions.Conjunction([])
            for term, param in termdict.iteritems():
                pre.parts.append(conditions.LiteralCondition(predicates.equals, [term, predicates.Term(param)]))
                vars.append(param)

        return sensors.Sensor(sensor.name, agents, args, vars, pre, sense, domain)
    

