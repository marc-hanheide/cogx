import itertools

import predicates, conditions, effects, actions, scope, visitors, translators, mapl
import mapltypes as types
import builtin

from parser import ParseError, UnexpectedTokenError
from mapltypes import Type, TypedObject, Parameter
from predicates import Predicate, Function
from builtin import t_object, t_boolean

p = Parameter("?f", types.FunctionType(t_object))
observed = Predicate("observed", [p, Parameter("?v", types.ProxyType(p)), ], builtin=True)

modal_predicates = [observed]

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
        
class Observation(scope.Scope):
    def __init__(self, name, agents, args, execution, precondition, effect, domain):
        scope.Scope.__init__(self, args+agents, domain)
        self.name = name
        self.agents = agents
        self.args = args
        self.execution = execution
        self.precondition = precondition
        self.effect = effect

    def to_pddl(self):
        str = ["(:observe %s" % self.name]
        indent = len("(:observe ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        agents = [Parameter(p.name, p.type) for p in self.agents]
        args = [Parameter(p.name, p.type) for p in self.args]
        
        o = Observation(self.name, agents, args, None, None, None, newdomain)

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
    
    @staticmethod
    def parse(it, scope):
        it.get(":observe")
        name = it.get().token.string

        it.get(":agent")
        agent = predicates.parse_arg_list(iter(it.get(list, "agent parameter")), scope.types)

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types, previous_params=agent)
            next = it.get()
        else:
            params = []
        
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
            
        sensable_atoms = []
        if observe.precondition:
            sensable_atoms += observe.precondition.visit(atom_visitor)
        if isinstance(observe.effect, effects.ConditionalEffect):
            sensable_atoms += observe.effect.condition.visit(atom_visitor)
        
        for a in domain.actions:
            match = can_observe(a)
            if not match:
                continue
            if match == True:
                #sensor with no relation to a concrete action
                #not yet supported
                continue
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
        
        dom = _domain.copy()
        dom.requirements.discard("partial-observability")

        for o in dom.observe:
            self.translate_observable(o, dom)

        dom.observe = None
        return dom

    def translate_problem(self, _problem):
        domain = self.translate_domain(_problem.domain)
        if domain == _problem.domain:
            return _problem
        p2 = problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        return p2
