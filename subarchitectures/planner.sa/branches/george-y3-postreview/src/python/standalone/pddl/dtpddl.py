import itertools, math
from itertools import chain
from collections import defaultdict

import parser, predicates, conditions, effects, actions, scope, visitors, translators, writer, problem, state, mapl, utils
import mapltypes as types
import builtin

from builder import Builder
from parser import ParseError, UnexpectedTokenError
from mapltypes import Parameter
from predicates import Predicate, Function, FunctionTerm, VariableTerm, ConstantTerm
from builtin import t_object, t_number, UNKNOWN

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
    domain.dt_rules = []

def observe_handler(it, domain):
    domain.observe.append(Observation.parse(it, domain))
    return True

def rule_handler(it, domain):
    it.get(":dtrule").token
    new_token = parser.Token(":action", 0, 0)
    it.element[0] = parser.Element(new_token)
    a = actions.Action.parse(iter(it.element), domain)
    domain.dt_rules += DTRule.from_action(a)
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
    ":observe" : observe_handler,
    ":dtrule" : rule_handler
    }

def copy_hook(self, result):
    result.percepts = self.percepts.copy()
    result.observe = [s.copy(result) for s in self.observe]
    result.dt_rules = [r.copy(result) for r in self.dt_rules]

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
    def write_term(self, term):
        if isinstance(term, (predicates.ConstantTerm, predicates.VariableTerm)):
            if term.get_type().equal_or_subtype_of(types.t_number):
                if isinstance(term.object.name, float):
                    return "%.16f" % term.object.name
                
        return writer.Writer.write_term(self, term)

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
    def __init__(self, name, args, conditions, variables, values, costs, domain):
        scope.Scope.__init__(self, args, domain)
        self.name = name
        self.args = args
        self.costs = costs
        self.conditions = [c.copy(new_scope=self) for c in conditions]
        self.variables = [(f, self.lookup(a)) for f,a in variables]
        self.values = [(self.lookup([p])[0], self.lookup(vals)) for p, vals in values]

        self.inst_func_init = False
        self.deps = set(sum((lit.visit(visitors.collect_functions) for lit in self.conditions), []))

    def depends_on(self, other):
        return any(f in self.deps for f,_ in other.variables)
    
    def copy(self, newdomain=None):
        """Create a deep copy of this Rule.

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        r = DTRule(self.name, [], [], [], [],  self.costs, newdomain)
        r.args = r.copy_args(self.args)
        
        r.conditions = [c.copy(new_scope=r) for c in self.conditions]
        r.variables = [(f, r.lookup(a)) for f,a in self.variables]
        r.values = [(r.lookup([p])[0], r.lookup(vals)) for p, vals in self.values]

        r.deps = set(sum((lit.visit(visitors.collect_functions) for lit in r.conditions), []))
        
        return r

    def match_args(self, facts):
        mapping = {}
        for fact in facts:
            for lit in self.conditions:
                lit_mapping = fact.match_literal(lit)
                if lit_mapping is None:
                    continue
                if any(mapping.get(a,val) != val for a, val in lit_mapping.iteritems()):
                    # conflict!
                    return None
                mapping.update(lit_mapping)
        return mapping


    def get_value_args(self):
        result = set()
        for t,v in self.values:
            if isinstance(v, predicates.VariableTerm):
                result.add(v.object)
        return result


    def prepare_inst_func(self):
        self.pcond = None
        if len(self.values) == 1 and not isinstance(self.values[0][0], predicates.ConstantTerm):
            self.pcond = conditions.LiteralCondition(builtin.gt, [self.values[0][0], predicates.Term(0)])

        self.lit_by_arg = defaultdict(set)
        self.free_args = {}

        for lit in self.conditions + [self.pcond]:
            if lit:
                free = lit.free()
                self.free_args[lit] = free
                for arg in free:
                    self.lit_by_arg[arg].add(lit)
        self.inst_func_init = True

    def get_inst_func(self, st):
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
                if lit in checked or lit is None:
                    continue
                # print lit.pddl_str()

                if lit.predicate == builtin.equals and isinstance(lit.args[0], FunctionTerm) and isinstance(lit.args[1], VariableTerm):
                    v = lit.args[-1]
                    if all(a.is_instantiated() for a in lit.args[0].args if isinstance(a, VariableTerm)) and  isinstance(v, VariableTerm) and not v.is_instantiated():
                        svar = state.StateVariable.from_literal(lit, st)
                        forced = v.object, st[svar]
                        forced_lit = lit

                if all(a.is_instantiated() for a in self.free_args[lit]):
                    if lit == self.pcond:
                        svar = state.StateVariable.from_literal(lit, st)
                        val = st[svar]
                        if val == builtin.UNKNOWN or val.value <= 0.001:
                            # print "failed:", svar, val
                            return None, None
                    else:
                        fact = state.Fact.from_literal(lit, st)
                        #cvar = state.StateVariable(t.function, state.instantiate_args(t.args))
                        exst = st.get_extended_state([fact.svar])
                        #val = st.evaluate_term(v)
                        if exst[fact.svar] != fact.value:
                            # print "failed:", fact, exst[fact.svar]
                            return None, None
                    checked.add(lit)
                else:
                    next_candidates.append([a for a in self.free_args[lit] if not a.is_instantiated()])

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
    
    @staticmethod
    def from_action(action):
        @visitors.collect
        def extract_terms_with_prob(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate in builtin.numeric_ops:
                    return
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

        conds = visitors.visit(action.precondition, visitors.collect_literals, [])
        
        values = defaultdict(list)
        for p, term, value in visitors.visit(action.effect, extract_terms_with_prob, []):
            values[p].append((term, value))

        cost_term = action.get_total_cost()

        rules = []
        for p, facts in values.iteritems():
            terms, vals = zip(*facts)
            variables = [(t.function, t.args) for t in terms]
            rules.append(DTRule(action.name, action.args[:], conds, variables, [(p, vals)], cost_term, action.parent))
        # for r in rules:
        #     print r
        return rules

    def __str__(self):
        cstr = ", ".join("%s" % lit.pddl_str() for lit in self.conditions)
        vstrs = []
        for p, vals in self.values:
            vstr = []
            for (f, args), v in zip(self.variables, vals):
                vstr.append("%s(%s) = %s" % (f.name, ", ".join(a.pddl_str() for a in args), v.pddl_str()))
            if p is None:
                vstrs.append("rest: %s" % ", ".join(vstr))
            else:
                vstrs.append("%s: %s" % (p.pddl_str(), ", ".join(vstr)))
                
        vstr = ", ".join(vstrs)
        s = "%s: (when %s) %s" % (self.name, cstr, vstr)
        return s
    

        
class DTRuleOld(scope.Scope):
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
        return other.function in self.deps()
    
    def copy(self, newdomain=None):
        """Create a deep copy of this Rule.

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        r = DTRule(self.function, [], [], [], [], newdomain, self.name)
        r.args = r.copy_args(self.args)
        r.add_args = r.copy_args(self.add_args)
        r.conditions = [c.copy(new_scope=r) for c in self.conditions]
        r.values = [tuple(r.lookup(c)) for c in self.values]
        
        return r

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
        self.free_args = {}

        for lit in self.conditions + [self.pcond]:
            if lit:
                free = lit.free()
                self.free_args[lit] = free
                for arg in free:
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
                # print lit.pddl_str()

                if lit.predicate == builtin.equals and isinstance(lit.args[0], FunctionTerm) and isinstance(lit.args[1], VariableTerm):
                    v = lit.args[-1]
                    if all(a.is_instantiated() for a in lit.args[0].args if isinstance(a, VariableTerm)) and  isinstance(v, VariableTerm) and not v.is_instantiated():
                        svar = state.StateVariable.from_literal(lit, st)
                        forced = v.object, st[svar]
                        forced_lit = lit

                if all(a.is_instantiated() for a in self.free_args[lit]):
                    if lit == self.pcond:
                        svar = state.StateVariable.from_literal(lit, st)
                        val = st[svar]
                        if val == builtin.UNKNOWN or val.value <= 0.001:
                            # print "failed:", svar, val
                            return None, None
                    else:
                        fact = state.Fact.from_literal(lit, st)
                        #cvar = state.StateVariable(t.function, state.instantiate_args(t.args))
                        exst = st.get_extended_state([fact.svar])
                        #val = st.evaluate_term(v)
                        if exst[fact.svar] != fact.value:
                            # print "failed:", fact, exst[fact.svar]
                            return None, None
                    checked.add(lit)
                else:
                    next_candidates.append([a for a in self.free_args[lit] if not a.is_instantiated()])

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

    def match_args(self, facts):
        mapping = {}
        for svar, val in facts:
            for lit in self.conditions:
                if lit.predicate == builtin.equals and lit.args[0].function == svar.function:
                    for a, a2 in zip(lit.args[0].args + [lit.args[1]], svar.args + (val,)):
                        if isinstance(a, VariableTerm):
                            assert mapping.get(a.object, a2) == a2
                            mapping[a.object] = a2
                        elif isinstance(a, ConstantTerm) and a.object != a2:
                            return None
                elif lit.predicate == svar.function:
                    v = builtin.TRUE if not lit.negated else builtin.FALSE
                    for a, a2 in zip(lit.args + [v], self.svar.args + (val,)):
                        if isinstance(a, VariableTerm):
                            assert mapping.get(a.object, a2) == a2
                            mapping[a.object] = a2
                        elif isinstance(a, ConstantTerm) and a.object != a2:
                            return None
        return mapping


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

            
            self.annotations[a.name] = set()
            
            #temporarily rename parameters
            renamings = {}
            for arg in observe.args:
                if arg.name in a:
                    oldname = arg.name
                    i=1
                    while arg.name in a:
                        observe.rename(arg, "%s%d" % (oldname, i))
                        i+=1
                    renamings[arg.name] = oldname
            
            mapping = dict(zip(match.args, a.args))
            observe.instantiate(mapping)
            for atom in sensable_atoms:
                assert atom.predicate in (builtin.equals, builtin.eq)
                term = atom.args[0]
                value = atom.args[1]
                for arg in term.args:
                    if isinstance(arg, predicates.VariableTerm) and not arg.is_instantiated():
                        # print a.name, "-", map(str, a.args), "-", map(str, a.vars)
                        new_arg = a.copy_args([arg.object])[0]
                        a.args.append(new_arg)
                        a.vars = a.vars + [new_arg]
                        mapping[arg] = new_arg
                        observe.uninstantiate()
                        observe.instantiate(mapping)
                        # print a.name, "-", map(str, a.args), "-", map(str, a.vars)
                        
                # if atom.predicate in (builtin.equals, builtin.eq):
                #     lhs = atom
                #     s_atom = atom.copy_instance()
                #     s_atom.set_scope(a)
                #     a.sensors.append(mapl.SenseEffect(s_atom, a))
                # else:
                    #Free parameter on rhs => fully observable
                if isinstance(value, predicates.VariableTerm) and not value.is_instantiated():
                    s_term = a.lookup([atom.args[0].copy_instance()])[0]
                    a.sensors.append(mapl.SenseEffect(s_term, a))
                # elif any(isinstance(a, types.Parameter) and not a.is_instantiated() for a in atom.visit(visitors.collect_free_vars)):
                else:
                    # print map(str, atom.visit(visitors.collect_free_vars))
                    s_atom = atom.copy_instance()
                    s_atom.set_scope(a)
                    a.sensors.append(mapl.SenseEffect(s_atom, a))
            observe.uninstantiate()
            for s in a.sensors:
                self.annotations[a.name].add(s)

            #undo renamings
            for name, oldname in renamings.iteritems():
                arg = observe[name]
                observe.rename(arg, oldname)

    def create_commit_actions(self, domain, prob_functions):
        import durative

        p_functions = [r.function for r in domain.dt_rules]

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
        
        for r in domain.dt_rules:
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
                        cparts.append(b.cond(mapl.commit, lit.args[0], lit.args[1]))
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
        if isinstance(action, mapl.MAPLAction):
            agent_term = predicates.VariableTerm(action.agents[0])

            @visitors.replace
            def condition_visitor(cond, results):
                if isinstance(cond, conditions.LiteralCondition):
                    if translators.get_function(cond) in prob_functions:
                        conds = [translators.set_modality(cond, mapl.commit),
                                 translators.set_modality(cond, mapl.knowledge, [agent_term])]
                        return conditions.Conjunction(conds)

            # dep_conditions = set()
            @visitors.replace
            def ceff_condition_visitor(cond, results):
                if isinstance(cond, conditions.LiteralCondition):
                    if translators.get_function(cond) in prob_functions:
                        conds = [translators.set_modality(cond, mapl.commit),
                                 translators.set_modality(cond, mapl.knowledge, [agent_term])]
                        # dep_conditions.add(translators.set_modality(cond.copy(), mapl.committed))
                        return conditions.Conjunction(conds)

            @visitors.replace
            def effect_visitor(eff, results):
                # if isinstance(eff, effects.SimpleEffect) and eff.predicate == builtin.num_assign:
                #     if eff.args[0].function == reward:
                #         term = FunctionTerm(builtin.total_cost, [])
                #         value = -eff.args[1].object.value + 1
                #         return effects.SimpleEffect(builtin.increase, [term, predicates.Term(value) ])
                if isinstance(eff, effects.ConditionalEffect):
                    eff.condition.visit(ceff_condition_visitor)

            visitors.visit(action.precondition, condition_visitor)
            visitors.visit(action.effect, effect_visitor)
        # if dep_conditions:
        #     action.effect = effects.ConjunctiveEffect.join([action.effect] + list(dep_conditions))

        started_pred = domain.predicates.get("started", [])
            
        if started_pred:
            action.effect = effects.ConjunctiveEffect.join([action.effect, effects.SimpleEffect(started_pred,[])])
            
        # if action.sensors:
        #     commit_cond = action.commit_condition()
        #     action.precondition = conditions.Conjunction.new(action.precondition)
        #     action.precondition.parts.append(commit_cond)
        return action
    
    @translators.requires('partial-observability')
    def translate_domain(self, _domain, prob_functions = set()):
        translators.Translator.get_annotations(_domain)['has_commit_actions'] = True
        #if 'observe_effects' not in translators.Translator.get_annotations(_domain):
        translators.Translator.get_annotations(_domain)['observe_effects'] = {}
        self.annotations = translators.Translator.get_annotations(_domain)['observe_effects']
            
        dom = _domain.copy()
        dom.requirements.discard("partial-observability")

        self.add_function(total_p_cost, dom)
        self.add_function(started, dom)

        for r in _domain.dt_rules:
            for func, _ in r.variables:
                prob_functions.add(func)
        
        for o in dom.observe:
            self.translate_observable(o, prob_functions, dom)

        actions = []
        for a in dom.actions:
            actions.append(self.translate_action(a, prob_functions, dom))
                
        # for f in dom.functions:
        #     if "p-%s" % f.name not in dom.functions:
        #         continue
        #     args = [predicates.Parameter(a.name, a.type) for a in f.args]
        #     varg = predicates.Parameter("?val", f.type)
        #     pfunc = dom.functions.get("p-%s" % f.name, args+[varg])
        #     if not pfunc:
        #         continue
        #     pterm = predicates.Term(pfunc, args+[varg])
        #     rule = DTRule(f, args, [varg], [], [(pterm, predicates.Term(varg))], dom)
        #     rules.append(rule)

        dom.clear_actions()
        commit_actions = self.create_commit_actions(dom, prob_functions)
        for a in chain(actions, commit_actions):
            dom.add_action(a)
                
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


class DT2MAPLCompilerFD(DT2MAPLCompiler):
    # def __init__(self, **kwargs):
    #     DT2MAPLCompiler.__init__(self, **kwargs)

    def create_commit_actions(self, domain, prob_functions):
        # assert self.pnodes

        self.add_function(probability, domain)
        self.add_function(selected, domain)
        if t_node.name not in domain.types:
            domain.types[t_node] = t_node
            domain.types[t_node_choice] = t_node_choice
            
        actions = []
        # for n in self.pnodes:
        #     n.prepare_actions()
        # for n in self.pnodes:
        #     actions += n.to_actions(domain)
            
        return actions + self.commit_actions_from_rules(domain, prob_functions)
    
    @translators.requires('partial-observability')
    def translate_problem(self, _problem):
        p2 = DT2MAPLCompiler.translate_problem(self, _problem)
        for c in p2.domain.constants:
            p2.remove_object(c)
            
        return p2
    
    def commit_actions_from_rules(self, domain, prob_functions):
        import durative

        if "probability" not in domain.functions:
            domain.functions.add(probability)
        
        actions = []
        action_count = defaultdict(lambda: 0)
        
        started_pred = domain.predicates.get("started", [])
        
        for r in domain.dt_rules:
            for p, values in r.values:
                agent = predicates.Parameter("?a", mapl.t_planning_agent)
                i = action_count[r]
                action_count[r] += 1
                a = mapl.MAPLAction("__commit-%s-%d" % (r.name,i), [agent], r.args, [], None, None, None, [], domain)
                b = Builder(a)
                cparts = []
                if started_pred:
                    cparts.append(b.cond('not', (started_pred,)));
                    
                for lit in r.conditions:
                    if lit.predicate == builtin.equals and lit.args[0].function in prob_functions:
                        cparts.append(b.cond(mapl.commit, lit.args[0], lit.args[1]))
                    else:
                        cparts.append(lit)

                commit_effects = []
                for (f, args), v in zip(r.variables, values):
                    commit_effects.append(b.effect(mapl.commit, b(f, *args), v))
                    cparts.append(b.cond("not", (mapl.committed, b(f, *args))))
                prob_eff = b.effect("assign", (probability,), p )
                
                a.precondition = conditions.Conjunction(cparts)
                a.effect = effects.ConjunctiveEffect(commit_effects + [prob_eff], a)
                if r.costs is None:
                    a.set_total_cost(0)
                else:
                    a.set_total_cost(r.costs)

                actions.append(a)
            
        return actions

class DTPDDLCompiler(translators.Translator):
    def __init__(self, copy=True, **kwargs):
        self.orig_domain = None
        self.depends = [translators.MAPLCompiler(**kwargs)]
        self.set_copy(copy)

    def is_pure_epistemic_action(self, action):
        @visitors.collect
        def eff_visitor(eff, results):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate not in (mapl.knowledge, mapl.direct_knowledge):
                    return eff
                
        if visitors.visit(action.effect, []):
            return False
        return True

    def translate_action(self, action, domain):
        cost_term = action.get_total_cost()
        if cost_term is None:
            return action.copy(newdomain=domain)
                
        a2 = action.copy(newdomain=domain)
        a2.set_total_cost(None)

        orig_action = self.orig_domain.get_action(action.name)
        fct = predicates.Term(mapl.failure_cost,[])
        fail_cost_term = orig_action.get_effects(fct)

        b = Builder(a2)

        try:
            new_reward_eff = b.effect("assign", ("reward",), predicates.Term(-cost_term.object.value))
        except:
            #FIXME: HACK!!!!! EVIL EVIL HACK!!!! JUST FOR TESTING
            new_reward_eff = b.effect("assign", ("reward",), -10)

        a2.effect = effects.ConjunctiveEffect.join([a2.effect, new_reward_eff])
        
        if fail_cost_term:
            fail_cost_term = fail_cost_term[0]
            done_fn = domain.predicates["done"]
            done_fn = done_fn[0] if done_fn else None
            
            # print "has failure cost term:", fail_cost_term
            # try:
            new_fail_reward_eff = b.effect("assign", ("reward",), predicates.Term(-fail_cost_term.object.value))
            # except:
            #     #FIXME: HACK!!!!! EVIL EVIL HACK!!!! JUST FOR TESTING
            #     new_fail_reward_eff = b.effect("assign", ("reward",), -10)
            cond = a2.precondition
            eff = a2.effect
            a2.precondition = b.cond("not", (done_fn,)) if done_fn else None
            a2.effect = None
            @visitors.replace
            def remove_fail_costs(eff, parts):
                if isinstance(eff, effects.SimpleEffect):
                    if utils.is_functional(eff) and eff.args[0] == fct:
                        return False
            @visitors.replace
            def remove_done_precond(cond, parts):
                if isinstance(cond, conditions.LiteralCondition):
                    if cond.predicate == done_fn:
                        return False
            eff.visit(remove_fail_costs)
            cond.visit(remove_done_precond)

            c1 = effects.ConditionalEffect(cond, eff)
            c2 = effects.ConditionalEffect(cond.negate(), new_fail_reward_eff)
            a2.effect = effects.ConjunctiveEffect([c1, c2])
        
        return a2

    def translate_domain(self, _domain):
        dom = _domain.copy_skeleton()
        self.orig_domain = self.get_original(_domain)
        # dom.types[t_integer.name] = t_integer
        # dom.types[t_double.name] = t_double

        # def transform(f):
        #     if f.type == builtin.t_number and not f.builtin:
        #         return Function(f.name, f.args, t_double)
        # self.change_functions(dom.functions, transform)
        
        translators.change_builtin_functions(dom.predicates, default_predicates)
        # dom.functions = scope.FunctionTable(f for f in dom.functions if f.type is not builtin.t_number)
        dom.functions = scope.FunctionTable(dom.functions)
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
        self.orig_domain = self.get_original(_problem).domain
        p2 = translators.Translator.translate_problem(self, _problem)
        p2.optimization = 'maximize'
        p2.opt_func = Builder(p2)('reward')

        def is_ungroundable(action):
            for a in action.args:
                if len(list(p2.get_all_objects(a.type))) == 0:
                    return True
            return False
        
        p2.domain.actions = [a for a in p2.domain.actions if not is_ungroundable(a)]
        p2.domain.observe = [a for a in p2.domain.observe if not is_ungroundable(a)]
        
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
