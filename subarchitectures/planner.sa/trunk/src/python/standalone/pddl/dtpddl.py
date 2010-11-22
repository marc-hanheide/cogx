import itertools, math
from itertools import chain
from collections import defaultdict

import predicates, conditions, effects, actions, scope, visitors, translators, writer, problem, state, mapl
import mapltypes as types
import builtin

from builder import Builder
from parser import ParseError, UnexpectedTokenError
from mapltypes import Parameter
from predicates import Predicate, Function, FunctionTerm, VariableTerm
from builtin import t_object, t_number

t_node = types.Type("node")
t_node_choice = types.Type("node_choice")
t_integer = types.Type("integer", [t_number])
t_double = types.Type("double", [t_number])

selected = Function("selected", [Parameter("?n", t_node)], t_node_choice)

p = Parameter("?f", types.FunctionType(t_object))
observed = Predicate("observed", [p, Parameter("?v", types.ProxyType(p))], builtin=True)

reward = Function("reward", [], t_number, builtin=True)
started = Predicate("started", [])

total_p_cost = Function("total-p-cost", [], t_number)
probability = Function("probability", [], t_number)
p_cost_value = 200

pddl_module = True

default_types = [t_node, t_node_choice]
default_predicates = [observed]
default_functions = [reward, selected]

def prepare_domain(domain):
    domain.observe = []
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

def copy_hook(self, result):
    result.percepts = self.percepts.copy()
    result.observe = [s.copy(result) for s in self.observe]

def copy_skel_hook(self, result):
    result.percepts = self.percepts.copy()

def add_hook(self, result, action):
    if isinstance(action, Observation):
        self.observe.append(action)

def clear_hook(self, result):
    self.observe = []

def get_action_hook(self, result):
    return result + self.observe

domain_hooks = {
    'copy' : copy_hook,
    'copy_skeleton' : copy_skel_hook,
    'add_action' : add_hook,
    'clear_actions' : clear_hook,
    'get_action_like' : get_action_hook
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
        if first.token == "or":
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
            if arg.name == "?duration":
                continue
            
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
        self._agents = [a.name for a in agents]
        self._params = [a.name for a in params]
        self.execution = execution
        
    def _get_args(self, names):
        adict = dict((a.name, a) for a in self.args)
        return [adict[n] for n in names]
    def _set_args(self, names, newvals):
        names[:] = [a.name for a in newvals]

    agents = property(lambda self: self._get_args(self._agents), lambda self, x: self._set_args(self._agents, x))
    params = property(lambda self: self._get_args(self._params), lambda self, x: self._set_args(self._params, x))

    # def to_pddl(self):
    #     str = ["(:observe %s" % self.name]
    #     indent = len("(:observe ")

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        args = [Parameter(p.name, p.type) for p in self.args]
        adict = dict((a.name, a) for a in args)
        agents = [adict[a.name] for a in self.agents]
        params = [a for a in args if a not in agents]
        
        o = Observation(self.name, agents, params, None, None, None, newdomain)
        o.args = o.copy_args(o.args)
        
        if self.precondition:
            o.precondition = self.precondition.copy(o)
        if self.execution is not None:
            o.execution = [ex.copy(o, newdomain) for ex in self.execution]
        if self.effect:
            o.effect = self.effect.copy(o)
        return o

    def copy_skeleton(self, newdomain=None):
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

        except StopIteration:
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
            const = [c for c in domain.constants if c not in (types.UNKNOWN, )]
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

class DTRule(scope.Scope):
    def __init__(self, function, args, add_args, conditions, values, domain, name=None):
        scope.Scope.__init__(self, args+add_args, domain)
        self.name = (name if name else function.name)
        self.function = function
        self.args = args
        self.add_args = add_args
        self.conditions = [c.copy(new_scope=self) for c in conditions]
        self.values = [tuple(self.lookup(c)) for c in values]
        self.inst_func_init = False

    def deps(self):
        funcs = sum((lit.visit(visitors.collect_functions) for lit in self.conditions), [])
        return set(f for f in funcs if f != self.function)

    def depends_on(self, other):
        for func in self.deps():
            if func == other.function:
                return True
        return False

    def instantiate(self, mapping, parent=None):
        """Instantiate the Parameters of this action.

        Arguments:
        mapping -- either a dictionary from Parameters to TypedObjects
        or a list of TypedObjects. In the latetr case, the list is
        assumed to be in the order of the Action's Parameters."""
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        scope.Scope.instantiate(self, mapping, parent)

    def instantiate_all(self, mapping, parent=None):
        """Instantiate the Parameters of this action.

        Arguments:
        mapping -- either a dictionary from Parameters to TypedObjects
        or a list of TypedObjects. In the latetr case, the list is
        assumed to be in the order of the Action's Parameters."""
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args+self.add_args, mapping))
        scope.Scope.instantiate(self, mapping, parent)

    def prepare_inst_func(self):
        self.pcond = None
        if len(self.values) == 1 and not isinstance(self.values[0][0], predicates.ConstantTerm):
            self.pcond = conditions.LiteralCondition(builtin.gt, [self.values[0][0], predicates.Term(0)])

        self.lit_by_arg = defaultdict(set)
        for lit in self.conditions + [self.pcond]:
            if lit:
                for arg in lit.free():
                    self.lit_by_arg[arg].add(lit)
        self.inst_func_init = True

    def get_inst_func(self, st, ignored_cond=None):
        import state
        def args_visitor(term, results):
            if isinstance(term, FunctionTerm):
                return sum(results, [])
            return [term]

        if not self.inst_func_init:
            self.prepare_inst_func()
                    
        prev_mapping = {}
        checked = set()
            
        def inst_func(mapping, args):
            next_candidates = []
            if checked:
                for k,v in prev_mapping.iteritems():
                    if mapping.get(k, None) != v:
                        checked.difference_update(self.lit_by_arg[k])
            prev_mapping.update(mapping)
            
            #print [a.name for a in mapping.iterkeys()]
            forced = None
            forced_lit = None
            for lit in self.conditions + [self.pcond]:
                if lit == ignored_cond or lit in checked or lit is None:
                    continue

                if lit.predicate == builtin.equals and isinstance(lit.args[0], FunctionTerm) and isinstance(lit.args[1], VariableTerm):
                    v = lit.args[-1]
                    if all(a.is_instantiated() for a in lit.args[0].args if isinstance(a, VariableTerm)) and  isinstance(v, VariableTerm) and not v.is_instantiated():
                        svar = state.StateVariable.from_literal(lit, st)
                        forced = v.object, st[svar]
                        forced_lit = lit

                if all(a.is_instantiated() for a in lit.free()):
                    if lit == self.pcond:
                        svar = state.StateVariable.from_literal(lit, st)
                        val = st[svar]
                        if val == builtin.UNKNOWN or val.value <= 0.001:
                            return None, None
                    else:
                        fact = state.Fact.from_literal(lit, st)
                        #cvar = state.StateVariable(t.function, state.instantiate_args(t.args))
                        exst = st.get_extended_state([fact.svar])
                        #val = st.evaluate_term(v)
                        if exst[fact.svar] != fact.value:
                            return None, None
                    checked.add(lit)
                else:
                    next_candidates.append([a for a in lit.free() if not a.is_instantiated()])

            if forced:
                checked.add(forced_lit)
                #print "Forced %s = %s" % (str(forced[0]), str(forced[1]))
                return forced
            if next_candidates:
                next_candidates = sorted(next_candidates, key=lambda l: len(l))
                #print "Next:", next_candidates[0][0]
                return next_candidates[0][0], None
            #print self, [a.get_instance().name for a in  args]
            return True, None
        return inst_func
        
    # def get_probability(self, args, value, parent=None):
    #     varg = None
    #     pterm = None
    #     for p, v in self.values:
    #         if isinstance(v, predicates.ConstantTerm) and v.object == value:
    #             pterm = p
    #             varg = v.object
    #             break
    #         elif isinstance(v, predicates.VariableTerm) and value.is_instance_of(v.get_type()):
    #             pterm = p
    #             varg = v.object
    #             break
    #     if varg is None:
    #         return predicates.Term(0)
    #     mapping = dict((param.name, c) for (param,c ) in zip(self.args, args))
    #     if isinstance(varg, Parameter):
    #         mapping[varg.name] = value
    #     self.instantiate(mapping, parent)
    #     result = p.copy_instance()
    #     self.uninstantiate()
    #     return result

    def get_value_args(self):
        result = set()
        for t,v in self.values:
            if isinstance(v, predicates.VariableTerm):
                result.add(v.object)
        return result
        
    @staticmethod
    def from_action(action):
        @visitors.collect
        def extract_terms_with_prob(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                assert eff.predicate in builtin.assignment_ops
                term = eff.args[0]
                value = eff.args[1]
                return (1, term, value)
            if isinstance(eff, effects.ProbabilisticEffect):
                res = []
                for p, results2 in results:
                    for p_old, term, value in results2:
                        if p_old == 1:
                            res.append((p, term, value))
                        else:
                            p_new = predicates.FunctionTerm(builtin.mult, [p, p_old])
                            res.append((p_new, term, value))
                return res

        @visitors.collect
        def extract_conditions(cond, results):
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate in (builtin.equals, builtin.eq):
                    term = cond.args[0]
                    value = cond.args[1]
                else:
                    term = predicates.FunctionTerm(cond.predicate, cond.args)
                    if cond.negated:
                        value = predicates.Term(builtin.FALSE)
                    else:
                        value = predicates.Term(builtin.TRUE)
                return (term, value)

        #conds = visitors.visit(action.precondition, extract_conditions, [])
        conds = visitors.visit(action.precondition, visitors.collect_literals, [])
        
        values = defaultdict(list)
        for p, term, value in visitors.visit(action.effect, extract_terms_with_prob, []):
            values[term].append((p, value))

        rules = []
        for term, tvals in values.iteritems():
            term_args = [a.object for a in term.args]
            other_args = set()
            rule_values = []
            for p, val in tvals:
                rule_values.append((p, val))
                other_args |= set(val.visit(collect_param_visitor))
                if p is not None:
                    other_args |= set(p.visit(collect_param_visitor))

            if len(rule_values) == 2 and rule_values[1][0] is None:
                #special case for p/1-p distributions
                rule_values[1] = (predicates.FunctionTerm(builtin.minus, [predicates.Term(1), rule_values[0][0]]), rule_values[1][1])
                
            rel_conditions = []
            changed = True
            while changed:
                changed = False
                for lit in conds:
                    if lit in rel_conditions:
                        continue
                    #cargs = set(t.visit(collect_param_visitor) + v.visit(collect_param_visitor))
                    cargs = lit.free()
                    if cargs & (set(term_args) | other_args):
                        other_args |= cargs
                        rel_conditions.append(lit)
                        changed = True
            rules.append(DTRule(term.function, term_args, list(other_args - set(term_args)), rel_conditions, rule_values, action.parent, name=action.name))
        return rules

    def __str__(self):
        cstr = ", ".join("%s" % lit.pddl_str() for lit in self.conditions)
        vstrs = []
        for p,v in self.values:
            if p is None:
                vstrs.append("rest: %s" % v.pddl_str())
            else:
                vstrs.append("%s: %s" % (p.pddl_str(), v.pddl_str()))
                
        vstr = ", ".join(vstrs)
        s = "%s: (%s %s) (when %s) %s" % (self.name, self.function.name, " ".join(a.name for a in self.args), cstr, vstr)
        return s

def collect_param_visitor(term, results):
    """This visitor collects all TypedObjects that occur in this Term
    (and it's children) and returns them as a list."""
    
    if isinstance(term, (predicates.VariableTerm)):
        return [term.object]
    if isinstance(term, predicates.FunctionTerm):
        return sum(results, [])
    return []
                                

class DT2MAPLCompiler(translators.Translator):
    def translate_observable(self, observe, prob_functions, domain=None):
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
                if elem.predicate != observed and translators.get_function(elem) in prob_functions:
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

    def create_commit_actions(self, rules, domain):
        import durative

        p_functions = [r.function for r in rules]

        actions = []
        action_count = defaultdict(lambda: 0)

        def logp(p):
            if isinstance(p, predicates.ConstantTerm):
                return predicates.Term(max(-math.log(p.object.value, 2), 0.1))
            logname = "log-%s" % p.function.name
            func = domain.functions.get(logname, p.function.args)
            if not func:
                func = predicates.Function(logname, [Parameter(a.name, a.type) for a in p.function.args], builtin.t_number)
                domain.functions.add(func)
            return predicates.Term(func, p.args)
        
        for r in rules:
            for p, v in r.values:
                agent = predicates.Parameter("?a", mapl.t_agent)
                i = action_count[r.function]
                action_count[r.function] += 1
                a = mapl.MAPLDurativeAction("select-%s-%d" % (r.function.name,i), [agent], r.args, r.add_args, [], None, None, None, [], domain)
                b = Builder(a)
                #dterm = b("*", (total_p_cost,), ("-", 1, p))
                dterm = b("*", (total_p_cost,), logp(p))
                a.duration.append(durative.DurationConstraint(dterm))
                #cparts = [b.cond('not', ('=', p, 0))]
                cparts = []
                for lit in r.conditions:
                    if lit.predicate == builtin.equals and lit.args[0].function in p_functions:
                        cparts.append(b.cond("hyp", lit.args[0], lit.args[1]))
                    else:
                        cparts.append(lit)
                #lock_cond = b.cond("not", ("select-locked",))
                a.precondition = conditions.Conjunction([durative.TimedCondition("start", conditions.Conjunction(cparts))], a)
                                                             #durative.TimedCondition("all", b.cond("not", ("started",))), \
#                                                             durative.TimedCondition("start", conditions.Conjunction([lock_cond]+cparts))], a)

                #acquire_lock = b.timed_effect("start", "select-locked")
                #release_lock = b.timed_effect("end", "not", ("select-locked",))
                commit_eff = b.timed_effect("end", "commit", b(r.function, *r.args), v)
                #decrease_eff = b.timed_effect("end", "decrease", (total_p_cost,), "?duration")
                #a.effect = effects.ConjunctiveEffect([acquire_lock, release_lock, commit_eff, decrease_eff], a)
                #a.effect = effects.ConjunctiveEffect([commit_eff, decrease_eff], a)
                a.effect = effects.ConjunctiveEffect([commit_eff], a)

                actions.append(a)
            
        return actions

    def translate_action(self, action, prob_functions, domain=None):
        # @visitors.replace
        # def condition_visitor(cond, results):
        #     if isinstance(cond, conditions.LiteralCondition):
        #         if translators.get_function(cond) in prob_functions:
        #             return translators.set_modality(cond, mapl.hyp) 

        # dep_conditions = set()
        # @visitors.replace
        # def ceff_condition_visitor(cond, results):
        #     if isinstance(cond, conditions.LiteralCondition):
        #         if translators.get_function(cond) in prob_functions:
        #             dep_conditions.add(translators.set_modality(cond.copy(), mapl.committed))
        #             return translators.set_modality(cond, mapl.hyp)
                
        # @visitors.replace
        # def effect_visitor(eff, results):
        #     if isinstance(eff, effects.SimpleEffect) and eff.predicate == builtin.num_assign:
        #         if eff.args[0].function == reward:
        #             term = FunctionTerm(builtin.total_cost, [])
        #             value = -eff.args[1].object.value + 1
        #             return effects.SimpleEffect(builtin.increase, [term, predicates.Term(value) ])
        #     if isinstance(eff, effects.ConditionalEffect):
        #         eff.condition.visit(ceff_condition_visitor)

        # visitors.visit(action.precondition, condition_visitor)
        # visitors.visit(action.effect, effect_visitor)
        # if dep_conditions:
        #     action.effect = effects.ConjunctiveEffect.join([action.effect] + list(dep_conditions))

        action.effect = effects.ConjunctiveEffect.join([action.effect, effects.SimpleEffect(started,[])])
            
        # if action.sensors:
        #     commit_cond = action.commit_condition()
        #     action.precondition = conditions.Conjunction.new(action.precondition)
        #     action.precondition.parts.append(commit_cond)
        return action
    
    @translators.requires('partial-observability')
    def translate_domain(self, _domain, prob_functions = set()):
        translators.Translator.get_annotations(_domain)['has_commit_actions'] = True
        #if 'observe_effects' not in translators.Translator.get_annotations(_domain):
        translators.Translator.get_annotations(_domain)['observe_effects'] = []
        #if 'dt_rules' not in translators.Translator.get_annotations(_domain):
        translators.Translator.get_annotations(_domain)['dt_rules'] = []
        self.annotations = translators.Translator.get_annotations(_domain)['observe_effects']
            
        dom = _domain.copy()
        dom.requirements.discard("partial-observability")

        self.add_function(total_p_cost, dom)
        self.add_function(started, dom)

        for o in dom.observe:
            self.translate_observable(o, prob_functions, dom)

        sample_functions = set()
        actions = []
        rules = []
        for a in dom.actions:
            if a.name.startswith("_sample_"):
                try:
                    new_rules = DTRule.from_action(a)
                    for r in new_rules:
                        sample_functions.add(r.function)
                    rules += new_rules
                except:
                    actions.append(self.translate_action(a, prob_functions))
            else:
                actions.append(self.translate_action(a, prob_functions))
                
        for f in dom.functions:
            if "p-%s" % f.name not in dom.functions:
                continue
            args = [predicates.Parameter(a.name, a.type) for a in f.args]
            varg = predicates.Parameter("?val", f.type)
            pfunc = dom.functions.get("p-%s" % f.name, args+[varg])
            if not pfunc:
                continue
            pterm = predicates.Term(pfunc, args+[varg])
            rule = DTRule(f, args, [varg], [], [(pterm, predicates.Term(varg))], dom)
            rules.append(rule)

        dom.clear_actions()
        commit_actions = self.create_commit_actions(rules, dom)
        for a in chain(actions, commit_actions):
            dom.add_action(a)
                
        dtrules = translators.Translator.get_annotations(_domain)['dt_rules']
        dtrules.extend(rules)

        dom.observe = []
        return dom

    def get_prob_functions(self, _problem):
        prob_functions = set()
        for i in _problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                prob_functions |= set(i.visit(visitors.collect_non_builtins))
        return prob_functions

    @translators.requires('partial-observability')
    def translate_problem(self, _problem):
        prob_functions = self.get_prob_functions(_problem)
        domain = self.translate_domain(_problem.domain, prob_functions)
        if self.copy:
            return problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        _problem.set_parent(domain)

        # b = Builder(p2)
        # logfuncs = dict((f.name, f) for f in p2.domain.functions if f.name.startswith("log-"))
        # pfuncs = dict((f, logfuncs["log-%s" % f.name]) for f in p2.domain.functions if "log-%s" % f.name in logfuncs)
        # print logfuncs.keys()
        # print map(str, pfuncs.iterkeys())
        # newinit = []
        # for i in p2.init:
        #     if i.predicate == builtin.num_equal_assign and i.args[0].function in pfuncs and i.args[1].object.value > 0:
        #         logterm = predicates.Term(pfuncs[i.args[0].function], i.args[0].args)
        #         newinit.append(b.init('=', logterm, -math.log(i.args[1].object.value, 2)))
                
        # p2.init += newinit
        # p2.init.append(b.init('=', (total_p_cost,), p_cost_value))
        return p2

class PNode(object):
    nodeid = 0
    actionid = 0
        
    def __init__(self, svar, children):
        self.svar = svar
        self.parent = None
        self.children = children
        for p, nodes, facts in self.children.itervalues():
            for n in nodes:
                n.parent = self
        
        self.branch_vars = self.get_branch_vars()

        #self.hash = hash(tuple(self.children)) #TODO make order invariant

    @staticmethod
    def from_effect(peff):
        def get_children(eff):
            if isinstance(eff, effects.ConjunctiveEffect):
                return eff.parts
            else:
                return [eff]
            
        obj = types.TypedObject("node%d" % PNode.nodeid, t_node)
        PNode.nodeid += 1
        svar = state.StateVariable(selected, [obj])
            
        children = {}
        p_total = 0
        for i, (p, eff) in enumerate(peff.effects):
            p = p.object.value
            facts = [state.Fact.from_literal(e) for e in get_children(eff) if isinstance(e, effects.SimpleEffect)]
            facts = dict((f.svar, f.value) for f in facts)
            nodes = [PNode.from_effect(e) for e in get_children(eff) if isinstance(e, effects.ProbabilisticEffect)]
            val = types.TypedObject("choice%d" % i, t_node_choice)
                
            children[val] = (p, nodes, facts)
            p_total += p

        assert p_total <= 1.0001, "p(%s) = %.4f" % (str(svar), p_total)
        return PNode(svar, children)
        

    def hash(self):
        hashes = []
        for val, (p, nodes, facts) in self.children.iteritems():
            hashes.append(hash((p, frozenset([n.hash() for n in nodes]), frozenset(facts.iteritems()))))
        return hash(frozenset(hashes))

    def consolidate(self, nodedict):
        for val, (p, nodes, facts) in self.children.iteritems():
            nodes = [n.consolidate(nodedict) for n in nodes]
            self.children[val] = (p, nodes, facts)
        
        if self.hash() in nodedict:
            #print "duplicate found"
            return nodedict[self.hash()]
        nodedict[self.hash()] = self
        return self

    def replace_branch_var(self, svar):
        self.svar = svar
        children_new = {}
        for val, (p, nodes, facts) in self.children.iteritems():
            newval = facts[svar]
            del facts[svar]
            children_new[newval] = (p, nodes, facts)
        self.children = children_new

    def get_branches_for_fact(self, fact):
        if fact.svar == self.svar:
            if fact.value in self.children:
                return [fact.value]
            return []
        result = []
        for var, (p, nodes, facts) in self.children.iteritems():
            if facts[fact.svar] == fact.value:
                result.append(var)
        return result

    def get_nodedicts(self, ndict=None, fdict=None):
        if ndict is None:
            ndict = {}
        if fdict is None:
            fdict = defaultdict(set)
        assert ndict.get(self.svar, self) == self, self.svar
        
        if self.svar in ndict:
            return
        ndict[self.svar] = self
        fdict[self.svar].add(self)
        for p, nodes, facts in self.children.itervalues():
            for svar, val in facts.iteritems():
                fdict[svar].add(self)
            for n in nodes:
                n.get_nodedicts(ndict, fdict)
        return ndict, fdict
            
    @staticmethod
    def consolidate_all(nodes):
        assert False
        nodedict = {}
        nodes = [n.consolidate(nodedict) for n in nodes]
        svardict = defaultdict(set)
        for n in nodedict.itervalues():
            for svar in n.branch_vars:
                svardict[svar].add(n)
        for svar, nset in svardict.iteritems():
            print svar, len(nset)
            if len(nset) == 1:
                n = nset.pop()
                n.replace_branch_var(svar)
        return nodes
        

    @staticmethod
    def simplify_all(nodes):
        noderesult = nodes[:]
        factresult = {}
        for n in nodes:
            dn, df, empty = n.simplify()
            noderesult += dn
            factresult.update(df)
            if empty:
                noderesult.remove(n)
        return noderesult, factresult

    def simplify(self):
        if not self.children:
            return [], {}, True
        
        #TODO propagate constant literal outwards?
        for val, (p, nodes, facts) in self.children.iteritems():
            for n in nodes:
                dn, df, empty = n.simplify()
                nodes += dn
                facts.update(df)
                if empty:
                    nodes.remove(n)
            if p >= 1.0:
                cleaned_facts = dict((svar, val) for svar, val in facts.iteritems() if svar.function != selected)
                return nodes, cleaned_facts, True
            
        return [], {}, False

    def size(self, selected_facts=None):
        if selected_facts is not None and self.svar not in selected_facts:
            #print "tree not evaluated:", self.svar
            return 1
        size = 0
        total_p = 0
        for val, (p, nodes, facts) in self.children.iteritems():
            total_p += p
            if selected_facts is not None and val not in selected_facts[self.svar]:
                #print "branch not evaluated:", self.svar, val
                continue
            bsize = 1
            for n in nodes:
                bsize *= n.size(selected_facts)
            size += bsize
            #print "evaluated:", self.svar, val, bsize
        if total_p < 1.0:
            size += 1
        #print "size of", self.svar, size
        return size

    @staticmethod
    def reduce_all(nodes, choices, limit):
        levels = defaultdict(lambda: -1)
        def get_level(node, level):
            cval = choices.get(node.svar, None)
            if cval and levels[node] < level:
                levels[node] = level
                p, nodes, facts = node.children[cval]
                choices.update(facts)
                for n in nodes:
                    get_level(n, level+1)

        def total_size(nodes, facts=None):
            return reduce(lambda x,y: x*y, [n.size(facts) for n in nodes], 1)
        
        def update(d, it):
            for svar, val in it:
                d[svar].add(val)

        for n in nodes:
            get_level(n, 0)
            
        nodes_by_level = defaultdict(set)
        for n, l in levels.iteritems():
            nodes_by_level[l].add(n)
        node_order = sorted(nodes_by_level.iteritems(), key = lambda (l,n): -l)
        
        selected = defaultdict(set)
        update(selected, choices.iteritems())
        node_queue = []
        # print "initial choices:", [str(state.Fact(s,v)) for s,v in choices.iteritems()]
        while node_order:
            #while total_size(nodes, selected) < limit:
            level, this_nodes = node_order.pop(0)
            node_queue = chain(*itertools.izip_longest(*[n.add_facts(choices) for n in this_nodes]))
            for added_facts in node_queue:
                next = selected.copy()
                update(next, added_facts)
                tsize = total_size(nodes, next)
                if tsize < limit:
                    #print "added: %s (%d)" % (map(str, added_facts), tsize)
                    selected = next
                else:
                    #print "skipped (size = %d)" % tsize
                    pass
        return selected
            
    def add_facts(self, choices):
        cval = choices.get(self.svar, None)
        if cval:
            p, nodes, facts = self.children[cval]
            for n in nodes:
                #print "subtrees of %s=%s" % (str(self.svar), str(cval));
                yield n.all_facts()
        for val, (p, nodes, facts) in self.children.iteritems():
            if val == cval:
                continue
            result = set(state.Fact(svar, val) for svar, val in facts.iteritems())
            result.add(state.Fact(self.svar, val))
            for n in nodes:
                result |= n.all_facts()
            # print "branch %s=%s" % (str(self.svar), str(val))
            yield result

    def get_level(self, choices, level=0):
        cval = choices.get(self.svar, None)
        if not cval:
            return [(self, -1)]
        
        p, nodes, facts = self.children[cval]
        
    
    def all_facts(self):
        result = set()
        for val, (p, nodes, facts) in self.children.iteritems():
            result.add(state.Fact(self.svar, val))
            result |= set(state.Fact(svar, val) for svar, val in facts.iteritems())
            for n in nodes:
                result |= n.all_facts()
        return result

    def reduce(self, choices, limit):
        cval = choices.get(self.svar, None)

        if not cval:
            #no further choices:
            return 1, []

        p, nodes, facts = self.children[cval]
        size = 1
        next = [n for n in nodes if n.svar in choices]
        branch_limit = limit/2 # final limit will be 2*size of one branch when using branch combining
        # heuristic: if we need to select n subtrees, the limit of each is the nth root of the combined limit.
        subtree_limit = branch_limit**(1/float(len(next))) 
        selected_facts = set(facts + [state.Fact(self.svar, cval)])
        for n in next:
            s, f = n.reduce(choices, subtree_limit)
            size *= s
            selected_facts |= f
        
        #have a look at parallel subtrees first:
        #TODO: which branch to select?
        alloc_size = size
        for n in nodes:
            if n in next:
                continue
            if alloc_size * n.size() < branch_limit:
                print "choosing ", n.svar
                selected_facts |= n.all_facts()
                alloc_size *= n.size()
                
        #add alternative branches
        for val, (p, nodes, facts) in self.children.iteritems():
            if val == cval:
                continue
            branch_size = 1
            branch_facts = set()
            for n in nodes:
                branch_size *= n.size()
                branch_facts |= n.all_facts()
            if alloc_size + branch_size < branch_limit:
                print "choosing ", self.svar, cval
                

    def get_branch_vars(self):
        c_values = None
        for p, nodes, facts in self.children.itervalues():
            if not facts:
                return []
            
            if c_values is None:
                c_values = dict((svar, set([val])) for svar, val in facts.iteritems())
            else:
                for svar, vals in c_values.items():
                    if svar not in facts or facts[svar] in vals:
                        del c_values[svar]
                    else:
                        vals.add(facts[svar])

            if not c_values:
                return []
            
        return c_values

    @staticmethod
    def from_problem(problem):
        nodes = []
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                nodes.append(PNode.from_effect(i))
        return nodes

    def prepare_actions(self):
        self.visited_by = set()
        for val, (p, nodes, facts) in self.children.iteritems():
            for n in nodes:
                n.prepare_actions()

    def to_actions(self, domain, parent_conds=None):
        actions = []
        # if parent_conds:
        #     if frozenset(parent_conds) in self.visited_by:
        #         return []
        #     self.visited_by.add(frozenset(parent_conds))

        for o in self.svar.args:
            domain.add_constant(o)
            
        for val, (p, nodes, facts) in self.children.iteritems():
            if p <= 0.00001:
                continue
            name = "commit-%s-%s-%s" % (self.svar.function.name, "-".join(a.name for a in self.svar.args), val.name)
            PNode.actionid += 1
            domain.add_constant(val)
                
            agent = predicates.Parameter("?a", mapl.t_agent)
            a = mapl.MAPLAction(name, [agent], [], [], conditions.Conjunction([]), None, effects.ConjunctiveEffect([]), [], domain)
            actions.append(a)
            b = Builder(a)
            a.precondition.parts.append(b.cond("not", ("committed", self.svar.as_term())))
            a.precondition.parts.append(b.cond("not", ("started",)))
            if parent_conds:
                a.precondition.parts += [c.copy(a) for c in parent_conds]

            a.effect.parts.append(b.effect("assign", self.svar.as_term(), val))
            for svar, value in facts.iteritems():
                for c in chain(svar.args, [value]):
                    domain.add_constant(c)
                cvar = svar.as_modality(mapl.commit, [value])
                a.effect.parts.append(cvar.as_literal(_class=effects.SimpleEffect))
                
            a.effect.parts.append(b.effect("assign", ("probability",), p))

            new_parent_cond = b.cond("=", self.svar.as_term(), val)

            for n in nodes:
                actions += n.to_actions(domain, [new_parent_cond])

        return actions

    def to_init(self, selected_facts=None):
        effs = []
        for val, (p, nodes, facts) in self.children.iteritems():
            ceff = effects.ConjunctiveEffect([])
            for svar, val in chain(facts.iteritems(), [(self.svar, val)]):
                #if svar.function == selected:
                #    continue
                f = state.Fact(svar, val)
                if selected_facts is not None and f not in selected_facts:
                    continue
                eff = f.as_literal(_class=effects.SimpleEffect)
                ceff.parts.append(eff)
            for n in nodes:
                eff = n.to_init(selected_facts)
                if eff:
                    ceff.parts.append(eff)
            if ceff.parts:
                effs.append((predicates.Term(p), ceff))
        if effs:
            return effects.ProbabilisticEffect(effs)
        return None

class DT2MAPLCompilerFD(DT2MAPLCompiler):
    def __init__(self, nodes=None, **kwargs):
        DT2MAPLCompiler.__init__(self, **kwargs)
        self.pnodes = nodes

    def create_commit_actions(self, rules, domain):
        assert self.pnodes

        self.add_function(probability, domain)
        self.add_function(selected, domain)
        if t_node.name not in domain.types:
            domain.types[t_node] = t_node
            domain.types[t_node_choice] = t_node_choice
            
        actions = []
        for n in self.pnodes:
            n.prepare_actions()
        for n in self.pnodes:
            actions += n.to_actions(domain)
        return actions
    
    @translators.requires('partial-observability')
    def translate_problem(self, _problem):
        p2 = DT2MAPLCompiler.translate_problem(self, _problem)
        for c in p2.domain.constants:
            p2.remove_object(c)
            
        return p2
    # def create_commit_actions_old(self, rules, domain):
    #     import durative

    #     if "probability" not in domain.functions:
    #         domain.functions.add(probability)
        
    #     p_functions = [r.function for r in rules]

    #     actions = []
    #     action_count = defaultdict(lambda: 0)
        
    #     for r in rules:
    #         for p, v in r.values:
    #             agent = predicates.Parameter("?a", mapl.t_agent)
    #             i = action_count[r.function]
    #             action_count[r.function] += 1
    #             a = mapl.MAPLAction("select-%s-%d" % (r.function.name,i), [agent], r.args, r.add_args, None, None, None, [], domain)
    #             b = Builder(a)
    #             cparts = []
    #             for lit in r.conditions:
    #                 if lit.predicate == builtin.equals and lit.args[0].function in p_functions:
    #                     cparts.append(b.cond("hyp", lit.args[0], lit.args[1]))
    #                 else:
    #                     cparts.append(lit)

    #             a.precondition = conditions.Conjunction(cparts)
    #             commit_eff = b.effect("commit", b(r.function, *r.args), v)
    #             prob_eff = b.effect("assign", ("probability",), p )
    #             a.effect = effects.ConjunctiveEffect([commit_eff, prob_eff], a)

    #             actions.append(a)
            
    #     return actions

class DTPDDLCompiler(translators.Translator):
    def __init__(self, copy=True, **kwargs):
        self.depends = [translators.MAPLCompiler(**kwargs)]
        self.set_copy(copy)

    def translate_action(self, action, domain):
        cost_term = action.get_total_cost()
        if cost_term is None:
            return action.copy(newdomain=domain)

        a2 = action.copy(newdomain=domain)
        a2.set_total_cost(None)
        b = Builder(a2)
        
        new_reward_eff = b.effect("assign", ("reward",), predicates.Term(-cost_term.object.value))
        if isinstance(a2.effect, effects.ConjunctiveEffect):
            a2.effect.parts.append(new_reward_eff)
        else:
            new_eff = b.con(a2.effect, new_reward_eff)
            a2.effect = new_eff
        return a2

    def translate_domain(self, _domain):
        dom = _domain.copy_skeleton()
        # dom.types[t_integer.name] = t_integer
        # dom.types[t_double.name] = t_double

        # def transform(f):
        #     if f.type == builtin.t_number and not f.builtin:
        #         return Function(f.name, f.args, t_double)
        # self.change_functions(dom.functions, transform)
        
        translators.change_builtin_functions(dom.predicates, default_predicates)
        dom.functions = scope.FunctionTable(f for f in dom.functions if f.type is not builtin.t_number)
        self.add_function(reward, dom)
        translators.change_builtin_functions(dom.functions, default_functions)

        self.add_function(builtin.increase, dom)
        self.add_function(builtin.decrease, dom)

        actions = _domain.get_action_like()
        dom.clear_actions()
        for a in actions:
            dom.add_action(self.translate_action(a, dom))

        dom.axioms = [self.translate_axiom(a, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        return dom

    def translate_problem(self, _problem):
        p2 = translators.Translator.translate_problem(self, _problem)
        p2.optimization = 'maximize'
        p2.opt_func = Builder(p2)('reward')
        return p2


class ProbADLCompiler(translators.ADLCompiler):
    def __init__(self, copy=True, **kwargs):
        self.depends = [translators.ObjectFluentCompiler(**kwargs), translators.CompositeTypeCompiler(**kwargs) ]
        self.set_copy(copy)

    def translate_action(self, action, domain=None):
        a2 = translators.Translator.translate_action(self, action, domain)
        a2.precondition = visitors.visit(a2.precondition, translators.ADLCompiler.condition_visitor)
        a2.replan = visitors.visit(a2.replan, translators.ADLCompiler.condition_visitor)
        return a2
        
    def translate_domain(self, _domain):
        dom = translators.Translator.translate_domain(self, _domain)
        #FIXME: the "sample_" prefix for dora collides with the rovers domain!!!
        dom.actions = [a for a in dom.actions if not a.name.startswith("_sample_")]
        dom.axioms = []
        dom.name2action = None
        return dom
    
    def translate_problem(self, _problem):
        dom = self.translate_domain(_problem.domain)
        p2 = problem.Problem(_problem.name, _problem.objects, [], _problem.goal, dom, _problem.optimization, _problem.opt_func)

        @visitors.copy
        def init_visitor(elem, results):
            if isinstance(elem, predicates.Literal):
                if elem.negated:
                    return False
                
        for i in _problem.init:
            lit = i.visit(init_visitor)
            if lit:
                lit.set_scope(p2)
                p2.init.append(lit)

        p2.goal = visitors.visit(p2.goal, translators.ADLCompiler.condition_visitor)
        return p2
    

compilers = {'mapl' : DT2MAPLCompiler}
