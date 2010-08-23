import itertools

import predicates, conditions, effects, actions, scope, visitors, translators, writer, domain, mapl
import mapltypes as types
import builtin

from builder import Builder
from parser import ParseError, UnexpectedTokenError
from mapltypes import Type, TypedObject, Parameter
from predicates import Predicate, Function
from builtin import t_object, t_boolean, t_number

p = Parameter("?f", types.FunctionType(t_object))
observed = Predicate("observed", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

p = Parameter("?f", types.FunctionType(t_object))
committed = Predicate("committed", [p], builtin=True)

reward = Function("reward", [], t_number, builtin=True)

pddl_module = True

default_predicates = [observed, committed]
default_functions = [reward]

def prepare_domain(domain):
    domain.percepts = scope.FunctionTable()

def observe_handler(it, domain):
    domain.observe.append(Observation.parse(it, domain))
    return True

def percept_handler(it, domain):
    it.get(":percepts")
    for elem in it:
        if elem.is_terminal():
            raise UnexpectedTokenError(elem, "predicate declaration")
        domain.percepts.add(predicates.Predicate.parse(iter(elem), domain.types))

    domain.predicates.add([p for p in domain.percepts])
    return True

parse_handlers = {
    ":percepts" : percept_handler,
    ":observe" : observe_handler
    }


class ExecutionCondition(object):
    def __init__(self, action, args, negated=False):
        self.action = action
        self.args = args
        self.negated = negated

    def copy(self, newparent=None, newdomain=None):
        if newparent:
            args = [newparent[a] for a in self.args]
        else:
            args = self.args[:]

        action = self.action
        if newdomain:
            action = newdomain.get_action(self.action.name)
        return ExecutionCondition(action, args, self.negated)
                
    @staticmethod
    def parse(it, scope, domain=None):
        first = it.get()
        if first.token == "and":
            result = []
            for elem in it:
                result += ExecutionCondition.parse(iter(elem), scope)
            return result
        
        negated = False
        if first.token == "not":
            negated = True
            it = iter(it.get(list, "execution condition"))
            first = it.get()
        
        name = first.token
        action = None
        if domain is None:
            domain = scope.parent
        for a in domain.actions:
            if a.name == name.string:
                action = a
        if not action:
            raise ParseError(name, "Unknown action: '%s'" % name.string)

        args = []
        for arg in action.args:
            next = it.get('terminal', "argument").token
            if next.string not in scope:
                raise ParseError(next, "Unknown identifier: '%s'" % next.string)
            ex_arg = scope[next.string]
            if not ex_arg.type.equal_or_subtype_of(arg.type):
                raise ParseError(next, "type of parameter '%s' (%s) is incompatible with argument '%s' (%s) of action '%s'" % (ex_arg.name, ex_arg.type.name, arg.name, arg.type.name, action.name))
            args.append(ex_arg)
        it.no_more_tokens()

        return [ExecutionCondition(action, args, negated)]
        
class Observation(actions.Action):
    def __init__(self, name, agents, params, execution, precondition, effect, domain):
        actions.Action.__init__(self, name, agents+params, precondition, effect, domain)
        self.agents = agents
        self.params = params
        self.execution = execution

    def to_pddl(self):
        str = ["(:observe %s" % self.name]
        indent = len("(:observe ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        args = [Parameter(p.name, p.type) for p in self.args]
        adict = dict((a.name, a) for a in args)
        agents = [adict[a.name] for a in self.agents]
        params = [a for a in args if a not in agents]
        
        o = Observation(self.name, agents, params, None, None, None, newdomain)

        for arg in o.args:
            if isinstance(arg.type, types.ProxyType):
                arg.type = types.ProxyType(o[arg.type.parameter])
        
        if self.precondition:
            o.precondition = self.precondition.copy(o)
        if self.execution is not None:
            o.execution = [ex.copy(o, newdomain) for ex in self.execution]
        if self.effect:
            o.effect = self.effect.copy(o)

        return o

    def copy_skeletion(self, newdomain=None):
        """Create a copy of this observation's skeleton (name,
        arguments but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        args = [Parameter(p.name, p.type) for p in self.args]
        adict = dict((a.name, a) for a in args)
        agents = [adict[a.name] for a in self.agents]
        params = [a for a in args if a not in agents]

        o = Observation(self.name, agents, params, None, None, None, newdomain)
        exe_cond = [ex.copy(newparent=o, newdomain=newdomain) for ex in self.execution]
        o.execution = exe_cond
        return o
    
    @staticmethod
    def parse(it, scope):
        it.get(":observe")
        name = it.get().token.string


        agent = []
        params = []
        
        next = it.get()
        if next.token.string == ":agent":
            agent = predicates.parse_arg_list(iter(it.get(list, "agent parameter")), scope.types)
            next = it.get()
            
        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types, previous_params=agent)
            next = it.get()
        
        observe = Observation(name, agent, params, None, None, None, scope)
        
        try:
            while True:
                if next.token.string == ":precondition":
                    if observe.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    observe.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), observe)
                elif next.token.string == ":execution":
                    if observe.execution:
                        raise ParseError(next.token, "execution condition already defined.")
                    observe.execution = ExecutionCondition.parse(iter(it.get(list, "execution condition")), observe)
                elif next.token.string == ":effect":
                    if observe.effect:
                        raise ParseError(next.token, "effects already defined.")
                    observe.effect = effects.Effect.parse(iter(it.get(list, "effect")), observe)
                else:
                    raise UnexpectedTokenError(next.token)
                next = it.next()

        except StopIteration, e:
            pass
        
        return observe

class DTPDDLWriter(writer.Writer):
    def write_observe(self, action):
        strings = [action.name]
        #TODO: this is really a bit of a hack...
        if "mapl" in action.parent.requirements:
            strings += self.section(":agent", ["(%s)" % self.write_typelist(action.agents)], parens=False)
            if action.params:
                strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.params)], parens=False)
        else:
            if action.args:
                strings += self.section(":parameters", ["(%s)" % self.write_typelist(action.args)], parens=False)
            

        exe = []
        for ex in action.execution:
            ex_str = "(%s %s)" % (ex.action.name, " ".join(a.name for a in ex.args))
            if ex.negated:
                ex_str = "(not %s)" % ex_str
            exe.append(ex_str)
        if len(exe) > 1:
            exe = [self.section("and", exe)]
            
        strings += self.section(":execution", exe, parens=False)
            
        if action.precondition:
            strings += self.section(":precondition", self.write_condition(action.precondition), parens=False)
        if action.effect:
            strings += self.section(":effect", self.write_effect(action.effect), parens=False)

            
        return self.section(":observe", strings)

    def write_percepts(self, preds):
        strings = []
        for pred in preds:
            if not pred.builtin:
                strings.append(self.write_predicate(pred))
        if not strings:
            return []
        return self.section(":percepts", strings)
    
    def write_domain(self, domain):
        strings = ["(define (domain %s)" % domain.name]
        strings.append("")
        strings.append("(:requirements %s)" % " ".join(":"+r for r in domain.requirements))
        strings.append("")
        strings += self.write_types(domain.types.itervalues())

        strings.append("")
        #HACK: this should be done in a more elegant way...
        strings += self.write_predicates([p for p in domain.predicates if not p.name.startswith("observed-")])

        strings.append("")
        #HACK: this should be done in a more elegant way...
        strings += self.write_percepts([p for p in domain.predicates if p.name.startswith("observed-")])
        
        strings.append("")
        strings += self.write_functions(domain.functions)
        
        if domain.constants:
            strings.append("")
            const = [c for c in domain.constants if c not in (types.TRUE, types.FALSE, types.UNKNOWN)]
            strings += self.write_objects("constants", const)

        for a in domain.axioms:
            strings.append("")
            strings += self.write_axiom(a)
            
        for a in domain.actions:
            strings.append("")
            strings += self.write_action(a)

        for o in domain.observe:
            strings.append("")
            strings += self.write_observe(o)
            

        strings.append("")
        strings.append(")")
        return strings


class DT2MAPLCompiler(translators.Translator):
    def translate_observable(self, observe, domain=None):
        assert domain is not None

        def can_observe(action):
            if not observe.execution:
                return True
            for ex in observe.execution:
                if ex.negated:
                    return ex.action != action
                if ex.action == action:
                    return ex
            return False

        @visitors.collect
        def atom_visitor(elem, results):
            if isinstance(elem, conditions.LiteralCondition):
                if elem.predicate != observed:
                    return [elem]

        @visitors.collect
        def effect_visitor(elem, results):
            if isinstance(elem, effects.ConditionalEffect):
                return elem.condition.visit(atom_visitor)
                
        sensable_atoms = []
        if observe.precondition:
            sensable_atoms += observe.precondition.visit(atom_visitor)
        sensable_atoms += observe.effect.visit(effect_visitor)

        for a in domain.actions:
            match = can_observe(a)
            if not match:
                continue
            if match == True:
                #sensor with no relation to a concrete action
                #not yet supported
                continue

            self.annotations.append(a.name)
            
            mapping = dict(zip(match.args, a.args))
            observe.instantiate(mapping)
            for atom in sensable_atoms:
                if atom.predicate not in (builtin.equals, builtin.eq):
                    s_atom = atom.copy_instance()
                    s_atom.set_scope(a)
                    a.sensors.append(mapl.SenseEffect(s_atom, a))
                else:
                    #Free parameter on rhs => fully observable
                    if isinstance(atom.args[1], predicates.VariableTerm) and not atom.args[1].is_instantiated():
                        s_term = a.lookup([atom.args[0].copy_instance()])[0]
                        a.sensors.append(mapl.SenseEffect(s_term, a))
                    else:
                        s_atom = atom.copy_instance()
                        s_atom.set_scope(a)
                        a.sensors.append(mapl.SenseEffect(s_atom, a))
                        
            observe.uninstantiate()
                
    
    def translate_domain(self, _domain):
        if "partial-observability" not in _domain.requirements:
            return _domain

        if 'observe_effects' not in translators.Translator.get_annotations(_domain):
            translators.Translator.get_annotations(_domain)['observe_effects'] = []
        self.annotations = translators.Translator.get_annotations(_domain)['observe_effects']
            
        dom = _domain.copy()
        dom.requirements.discard("partial-observability")

        for o in dom.observe:
            self.translate_observable(o, dom)

        dom.observe = []
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        if domain == _problem.domain:
            return _problem
        p2 = problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        return p2

class DTPDDLCompiler(translators.Translator):
    def __init__(self, **kwargs):
        self.depends = [translators.MAPLCompiler(**kwargs)]

    def translate_action(self, action, domain):
        cost_term = action.get_total_cost()
        if cost_term is None or cost_term == 0 :
            return action.copy(newdomain=domain)

        a2 = action.copy(newdomain=domain)
        b = Builder(a2)
        
        new_reward_eff = b.effect("decrease", ("reward",), cost_term)
        if isinstance(a2.effect, effects.ConjunctiveEffect):
            a2.effect.parts.append(new_reward_eff)
        else:
            new_eff = b.con(a2.effect, new_reward_eff)
            a2.effect = new_eff

        return a2
        
    def translate_domain(self, _domain):
        if "partial-observability" not in _domain.requirements:
            return _domain
        
        dom = domain.Domain(_domain.name, _domain.types.copy(), _domain.constants.copy(), _domain.predicates.copy(), _domain.functions.copy(), [], [])
        dom.requirements = _domain.requirements.copy()

        for func in default_predicates:
            dom.predicates.remove(func)
            
        p = Parameter("?f", types.FunctionType(t_object))
        observed = Predicate("observed", [p, Parameter("?v", types.ProxyType(p)), ], builtin=False)
        p = Parameter("?f", types.FunctionType(t_object))
        committed = Predicate("committed", [p], builtin=False)

        dom.predicates.add(observed)
        dom.predicates.add(committed)
        try:
            dom.predicates.add(builtin.increase)
        except:
            pass
        dom.predicates.add(builtin.decrease)

        dom.actions = [self.translate_action(a, dom) for a in _domain.actions]
        dom.observe = [self.translate_action(o, dom) for o in _domain.observe]
        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        dom.name2action = None
        return dom

