import sys
from collections import defaultdict
import itertools
import networkx

import standalone.globals as global_vars

from standalone import config, pddl, plans
from standalone.pddl import Predicate, Function, Parameter, Term, FunctionTerm, ConstantTerm
from standalone.pddl import visitors

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

FACT_TYPE_STATIC = 1
FACT_TYPE_STATIC_EXCLUSIVE = 2
FACT_TYPE_FLUENT = 3
FACT_TYPE_FLUENT_EXCLUSIVE = 4
FACT_TYPE_IRRELEVANT = 5

existence = Predicate("exists", [Parameter("?o", pddl.t_object)])

def abstract_match(abstract, other):
    if other is None:
        return True
    if abstract is None:
        return False
    
    if isinstance(other, Parameter):
        if isinstance(abstract, pddl.TypedObject):
            return abstract.isInstanceOf(other.type)
        return abstract.equalOrSubtypeOf(other.type)
    return abstract == other

def abstract_subset(abstract, other):
    if other is None:
        return True
    if abstract is None:
        return False
    
    if isinstance(other, pddl.TypedObject):
        return abstract == other
    if isinstance(abstract, pddl.TypedObject):
        return abstract.isInstanceOf(other)
    return abstract.equalOrSubtypeOf(other)

class VariableBinding(object):
    pass

class AbstractVariable(object):
    def __init__(self, function, args, modality=None, value=None):
        self.function = function
        self.modality = modality
        self.real_args = args
        self.args = []
        for a in args:
            if isinstance(a, pddl.FunctionTerm):
                a = pddl.VariableTerm(Parameter("?%s" % a.function.name, a.function.type))
            self.args.append(a)
        self.value = value

        self.hash = hash(self.signature())
        
    @staticmethod
    def from_atom(atom):
        modality=None
        if atom.predicate in [pddl.equals] + pddl.assignment_ops + pddl.numeric_ops + pddl.builtin.numeric_comparators:
            func = atom.args[0].function
            args = atom.args[0].args
            val = atom.args[1]
        elif any(isinstance(a.type, pddl.types.FunctionType) for a in atom.predicate.args):
            #TODO: make this more rigorous
            modality = atom.predicate
            func = atom.args[0].function
            args = atom.args[0].args
            val = atom.args[1]
        else:
           func = atom.predicate
           args = atom.args
           if atom.negated:
               val = pddl.FALSE
           else:
               val = pddl.TRUE

        return AbstractVariable(func, args, modality, val)

    def signature(self):
        sig = [self.function, self.modality]
        for a in self.args:
            if isinstance(a, pddl.VariableTerm):
                sig.append(a.get_type())
            elif isinstance(a, pddl.ConstantTerm):
                sig.append(a.object)
            else:
                assert False
        return tuple(sig)

    def really_equal(self, other):
        return self.function == other.function and self.modality == other.modality and all(a == o for a,o in zip(self.real_args, other.real_args))
    
    def subsumed_by(self, other):
        if self.function != other.function or self.modality != other.modality:
            return False
        for a, o in zip(self.args, other.args):# + [(self.value, other.value)]:
            if isinstance(o, pddl.ConstantTerm) and isinstance(a, pddl.VariableTerm):
                return False # constant can never subsume variable
            if isinstance(o, pddl.ConstantTerm) and isinstance(a, pddl.ConstantTerm):
                if o != a:
                    return False # constants are only compatible if they are equal
            if isinstance(o, pddl.VariableTerm) and isinstance(a, pddl.ConstantTerm):
                if not a.object.is_instance_of(o.get_type()):
                    return False # a is not of type o
                
            if not a.get_type().equal_or_subtype_of(o.get_type()):
                return False # a.type is not subsumed by o.type
        return True

    def __str__(self):
        if self.modality:
            return "(%s (%s %s))" % (self.modality.name, self.function.name, " ".join(str(a.object.name) for a in self.args))
        return "(%s %s)" % (self.function.name, " ".join(str(a.object.name) for a in self.args))
    
    def __hash__(self):
        return self.hash
    
    def __eq__(self, other):
        return isinstance(other, type(self)) and other.hash == self.hash
        
# class cgElement(tuple):
#     def __new__(_class, function, types, value=None):
#         assert value != True and value != False
#         #assert value.isInstanceOf(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
#         return tuple.__new__(_class, (function, tuple(types), value))
    
#     function = property(lambda self: self[0])
#     types = property(lambda self: self[1])
#     value = property(lambda self: self[2])

#     @staticmethod
#     def from_tuple(func, args, value):
#         if isinstance(value, Parameter):
#             val = value.type
#         else:
#             val = value
#         return cgElement(func, [a.type for a in args], val)
        
        
#     @staticmethod
#     def from_atom(atom):
#         if atom.predicate in pddl.assignment_ops:
#             func = atom.args[0].function
#             args = atom.args[0].args
#             if isinstance(atom.args[1], pddl.VariableTerm):
#                 val = atom.args[1].getType()
#             else:
#                 val = atom.args[1].object
#         else:
#             func = atom.predicate
#             args = atom.args
#             if atom.negated:
#                 val = pddl.FALSE
#             else:
#                 val = pddl.TRUE
#         return cgElement(func, [a.getType() for a in args], val)
            
#     def subset_of(self, other):
#         if self.function == other.function and all(map(lambda a,b: a.equalOrSubtypeOf(b), self.types, other.types)):
#             if isinstance(self.value, pddl.Type) and isinstance(other.value, pddl.Type):
#                 return self.value.equalOrSubtypeOf(other.value)
#             elif isinstance(self.value, pddl.TypedObject) and isinstance(other.value, pddl.Type):
#                 return self.value.isInstanceOf(other.value)
#             return self.value == other.value
#         return False

#     def subset_ignore_value(self, other):
#         return self.function == other.function and all(map(lambda a,b: a.equalOrSubtypeOf(b), self.types, other.types))
    
#     def threatened_by(self, other):
#         if self.function != other.function or not all(map(lambda a,b: a.equalOrSubtypeOf(b) or b.equalOrSubtypeOf(a), self.types, other.types)):
#             return False
#         return not self.subset_of(other)

#     def matches(self, atom):
#         return cgElement.from_atom(atom).subset_of(self)

#     def __str__(self):
#         if self.value is None or self.value == pddl.TRUE:
#             return "(%s %s)" % (self.function.name, " ".join(map(str, self.types)))
#         elif self.value == pddl.FALSE:
#             return "(not (%s %s))" % (self.function.name, " ".join(map(str, self.types)))
#         else:
#             return "(= (%s %s) %s)" % (self.function.name, " ".join(map(str, self.types)), self.value.name)

BINDING_NONE = 0
BINDING_DIRECT = 1
BINDING_INDIRECT = 2

class BindingGraph(object):
    def __init__(self, action):
        self.action = action
        self.edges = defaultdict(lambda: defaultdict(list))
        #self.conditions = set()
        #self.effects = set()
        self.compute()
        # for x, ysets in self.edges.iteritems():
        #     for yset, terms in ysets.iteritems():
        #         for t in terms:
        #             print "%s => %s" % (x.pddl_str(), t.pddl_str())
                
    def compute(self):
        all_args = self.action.args[:]
        
        def condition_visitor(cond, results):
            if isinstance(cond, pddl.LiteralCondition):
                if cond.predicate in (pddl.equals, pddl.builtin.eq):
                    assert isinstance(cond.args[0], pddl.FunctionTerm) and isinstance(cond.args[1], (pddl.ConstantTerm, pddl.VariableTerm))
                    if isinstance(cond.args[1], pddl.ConstantTerm):
                        return
                    self.edges[cond.args[1]][frozenset(cond.args[0].args)].append(cond.args[0])
            elif isinstance(cond, pddl.conditions.QuantifiedCondition):
                all_args.extend(cond.args)
            
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.ConditionalEffect):
                eff.condition.visit(condition_visitor)
            elif isinstance(eff, pddl.UniversalEffect):
                all_args.extend(eff.args)

        visitors.visit(self.action.precondition, condition_visitor)
        visitors.visit(self.action.effect, effect_visitor)

        for arg in all_args:
            self.edges[Term(arg)][frozenset([Term(arg)])] = []

        #for k, valsets in self.edges.iteritems():
        #    for vals in valsets.iterkeys():
        #        print "%s => (%s)" % (k.object.name, ", ".join(v.object.name for v in vals))

        # Computes the transitive closure (yes, this is horrible)
        changed = True
        while changed:
            changed = False
            for x, ysets in self.edges.iteritems():
                new = {}
                for yset, terms in ysets.iteritems():
                    for y in yset:
                        for zset, terms2 in self.edges[y].iteritems():
                            newval = frozenset((yset - set([y])) | zset)
                            if newval not in self.edges[x] and newval not in new:
                                newterms = []
                                for t1 in terms:
                                    for t2 in terms2:
                                        t_args = []
                                        for arg in t1.args:
                                            if arg == y:
                                                t_args.append(t2)
                                            else:
                                                t_args.append(arg)
                                        newterms.append(pddl.FunctionTerm(t1.function, t_args))
                                new[newval] = newterms
                                changed = True
                assert x in self.edges
                self.edges[x].update(new)

    def get_binding(self, var1, mapping):
        if not isinstance(mapping, dict):
            mapping = dict((a,a) for a in mapping)
        keys = set(mapping.keys())
        
        if var1 in mapping:
            return BINDING_DIRECT, []

        smallest = None
        for zset in self.edges[var1].iterkeys():
            if keys & zset:
                if smallest is None or zset - mapping < smallest - mapping:
                    smallest = zset
                    
        if smallest:
            def mapping_replace_visitor(term, results):
                if isinstance(term, pddl.FunctionTerm):
                    return pddl.FunctionTerm(term.function, results)
                return mapping.get(term, term)
            
            terms = []
            for term in self.edges[var1][smallest]:
                terms.append(term.visit(mapping_replace_visitor))
            return BINDING_INDIRECT, terms
        return BINDING_NONE, []

    def all_bindings(self):
        for x, ysets in self.edges.iteritems():
            for yset, terms in ysets.iteritems():
                for t in terms:
                    yield x,t
    
class CGEdge(object):
    def __init__(self, bgraph, cond, eff, prob=False, c_eff=False):
        self.action = bgraph.action
        self.probabilistic = prob
        self.conditional = c_eff
        self.observation = isinstance(self.action, pddl.dtpddl.Observation)
        self.rule = isinstance(self.action, pddl.mapl.InitRule)
        self.graph = bgraph
        self.cond = cond
        self.eff = eff
        assert cond != eff, "condition and effect are indistiguishable!"
        #self.fwd_map = []
        cbindings,_ = self.bind_vars(self.cond, self.eff)
        ebindings,_ = self.bind_vars(self.eff, self.cond)
        print "%s: %s => %s" % (self.action.name, str(cond), print_var(eff,ebindings))
        print "%s: %s => %s" % (self.action.name, print_var(cond,cbindings), str(eff))
        # for a in cond.args:
        #     if a in eff.real_args:
        #         self.fwd_map.append(eff.real_args.index(a))
        #     else:
        #         self.fwd_map.append(None)
        # print self.fwd_map
        # self.back_map = []
        # for a in eff.real_args:
        #     if a in cond.real_args:
        #         self.back_map.append(cond.real_args.index(a))
        #     else:
        #         self.fwd_map.append(None)

    def bind_vars(self, var1, var2, mapping=None, preferred=None):
        if self.eff == var1:
            own1 = self.eff
            own2 = self.cond
        elif self.eff == var2:
            own1 = self.cond
            own2 = self.eff
        else:
            assert False

        var_to_own = dict((a,a2) for a, a2 in zip(var2.args, own2.args))
        own_to_var = dict((a2,a) for a, a2 in zip(var1.args, own1.args))

        if mapping is None:
            mapping = dict((a, a) for a in own2.args)
        else:
            for a, a2 in zip(var2.args, own2.args):
                mapping[a2] = mapping.get(a,a2)
        
        preferred_map = mapping
        if preferred is not None:
            preferred_map = {}
            #print "#",[v.pddl_str() for v in preferred]
            #print "#",[v.pddl_str() for v in var_to_own.keys()]
            preferred_map = set((var_to_own[p], mapping[p]) for p in preferred)
        
        result = {}
        newpref = set()
        for a in own1.args:
            if isinstance(a, pddl.ConstantTerm):
                result[a] = a
            else:
                b, terms = self.graph.get_binding(a, preferred_map)
                pref = True
                if b == BINDING_NONE:
                    #try again with all values
                    b, terms = self.graph.get_binding(a, mapping)
                    pref = False
                    
                if b == BINDING_INDIRECT:
                    result[a] = terms[0]
                elif b == BINDING_DIRECT:
                    result[a] = mapping[a]
                else:
                    result[a] = a
                if pref:
                    newpref.add(a)

        for a, a2 in zip(var1.args, own1.args):
            result[a] = result[a2]
        newpref = set(own_to_var[p] for p in newpref)
        
        return result, newpref

    def get_effect_var(self, var):
        assert var.function == self.eff.function and var.modality == self.eff.modality
        mapping = dict((a,b) for a,b in zip(var.args, self.eff.args))
        def mapping_replace_visitor(term, results):
            if isinstance(term, pddl.FunctionTerm):
                return pddl.FunctionTerm(term.function, results)
            return mapping[term]

        return AbstractVariable()
    
    def __str__(self):
        prefix = ""
        if self.probabilistic:
            prefix += "P"
        if self.conditional:
            prefix += "C"
        if self.observation:
            prefix += "O"
        if self.rule:
            prefix += "R"
        if prefix:
            prefix += "-"
        return prefix+self.action.name
        
class CausalGraph(networkx.MultiDiGraph):
    def __init__(self, domain):
        networkx.MultiDiGraph.__init__(self)
        #self.problem = problem
        self.domain = domain
        #self.depths = {}
        #self.fact_types = defaultdict(lambda: FACT_TYPE_IRRELEVANT)
        #self.cooccuring = {}

        self.build_cg(self.domain)
        #self.calculate_types()
        

    def build_cg(self, domain):
        cg = self

        @visitors.collect
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.SimpleEffect):
                return ([], False, AbstractVariable.from_atom(eff))
            elif isinstance(eff, pddl.ConditionalEffect):
                newconds = eff.condition.visit(condition_visitor)
                res = []
                for cond, prob, eff in itertools.chain(*results):
                    res.append((cond+newconds, prob, eff))
                return res
            elif isinstance(eff, pddl.effects.ProbabilisticEffect):
                res = []
                for p, result in results:
                    for cond, prob, eff in itertools.chain(result):
                        res.append((cond, True, eff))
                return res

        @visitors.collect
        def condition_visitor(cond, results):
            if isinstance(cond, pddl.LiteralCondition):
                return AbstractVariable.from_atom(cond)

        actions = domain.actions + domain.init_rules + domain.observe
            
        for a in actions:
            bgraph = BindingGraph(a)
            conds = set(visitors.visit(a.precondition, condition_visitor, []))
            for econds, prob, eff in visitors.visit(a.effect, effect_visitor, []):
                for c in conds | set(econds):
                    if c.really_equal(eff):
                        continue
                    c_eff = c in econds
                    cg.add_edge(c, eff, cgedge=CGEdge(bgraph, c, eff, prob=prob, c_eff=c_eff) )

        # for k, values in cg.iteritems():
        #     for k2, values2 in cg.iteritems():
        #         if k2.subsumed_by(k):
        #             for k3,v in cg.iteritems():
        #                 if k in v:
        #                     cg[k3][k2] = set(cg[k3][k])

        #             for k3,v in values.iteritems():
        #                 if k3 not in values2:
        #                     values2[k3] = v
        #                 else:
        #                     values2[k3] |= v

        # changed = True
        # while changed:
        #     new_cg = {}
        #     changed = False
        #     for val in cg.itervalues():
        #         for k in val:
        #             if k not in cg:
        #                 for k2, v in cg.iteritems():
        #                     if  k.subsumed_by(k2):
        #                         mapping = dict(zip(k2.args, k.args))
        #                         v2 = {}
        #                         for k3, action in v.iteritems():
        #                             v2[AbstractVariable(k3.function, [mapping.get(t, t) for t in k3.types], k3.value)] = action
        #                         new_cg[k] = v2
        #                         changed = True
        #     cg.update(new_cg)

        for c in cg.nodes_iter():
            succ = cg.successors(c)
            if succ:
                print c
                for eff in succ:
                    actions = cg.actions(c, eff)
                    print "    %s: %s" % (str(eff), " ".join(map(str,actions)))#, map(str, f.args)

        return cg

    def actions(self, cond, eff):
        data = self.get_edge_data(cond, eff)
        if not data:
            return []
        return [d['cgedge'] for d in data.itervalues()]
        

    def get_probabilistic_vars(self, init_prob):
        #variables derived (non)deterministically from uncertain values
        prob = set(init_prob)
        closed = set()
        open = set(self.nodes_iter())
        while open:
            next = open.pop()
            closed.add(next)
            for c,eff,data in self.out_edges([next], data=True):
                edge = data['cgedge']
                if eff not in prob and edge.probabilistic:
                    prob.add(eff)
                    open.add(eff)
                    closed.discard(eff)
                elif eff not in prob and next in prob:
                    prob.add(eff)
                    open.add(eff)
        return prob

    def reachable(self, var, edge_func=lambda e,rev: rev):
        open = set([var])
        closed = set()
        while open:
            next = open.pop()
            closed.add(next)
            for cond, eff, data in self.out_edges_iter([next], data=True):
                if eff in closed:
                    continue
                if edge_func(data['cgedge'], False):
                    open.add(eff)
            for cond, eff, data in self.in_edges_iter([next], data=True):
                if cond in closed:
                    continue
                if edge_func(data['cgedge'], True):
                    open.add(cond)
        return closed

    
    def reachable_instances(self, var, edge_func=lambda e,rev: rev):
        init_mapping = dict((a,a) for a in var.args)
        open = {(var, frozenset(var.args)) : init_mapping}
        closed = {}
        while open:
            (next, pref), mapping = open.popitem()
            closed[next, pref] = mapping
            for cond, eff, data in self.out_edges_iter([next], data=True):
                #next == cond
                e = data['cgedge']
                mnew, newpref = e.bind_vars(eff, cond, mapping, pref)
                if (eff, frozenset(newpref)) in closed:
                    continue
                if edge_func(e, False):
                    open[eff, frozenset(newpref)] = mnew

            for cond, eff, data in self.in_edges_iter([next], data=True):
                #next == eff
                e = data['cgedge']
                mnew, newpref = e.bind_vars(cond, eff, mapping, pref)
                if (cond, frozenset(newpref)) in closed:
                    continue
                if edge_func(e, True):
                    open[cond, frozenset(newpref)] = mnew
                    
        del closed[var, frozenset(var.args)]
        return closed
    
    def calculate_types(self):
        fluents = set()
        dependencies = set()
        for elem, dep  in self.iteritems():
            fluents.add(elem)
            for el2 in dep.iterkeys():
                dependencies.add(el2)
                self.fact_types[el2] = FACT_TYPE_STATIC

        for elem in dependencies:
            if self.fact_types[elem] == FACT_TYPE_STATIC_EXCLUSIVE:
                continue
            if elem.value is None:
                self.fact_types[elem] = FACT_TYPE_STATIC_EXCLUSIVE
                continue
            for el2 in dependencies:
                if elem.threatened_by(el2):
                    self.fact_types[elem] = FACT_TYPE_STATIC_EXCLUSIVE
                    self.fact_types[el2] = FACT_TYPE_STATIC_EXCLUSIVE

        for elem in fluents:
            if elem.value is None:
                self.fact_types[elem] = FACT_TYPE_FLUENT_EXCLUSIVE
                continue
            for el2 in dependencies:
                if elem.threatened_by(el2):
                    self.fact_types[el2] = FACT_TYPE_FLUENT_EXCLUSIVE
                    if elem in dependencies:
                        self.fact_types[elem] = FACT_TYPE_FLUENT_EXCLUSIVE
                    else:
                        self.fact_types[elem] = FACT_TYPE_IRRELEVANT
                        
            if self.fact_types[elem] not in (FACT_TYPE_FLUENT_EXCLUSIVE, FACT_TYPE_IRRELEVANT):
                self.fact_types[elem] = FACT_TYPE_FLUENT
                
            
        for el, t in self.fact_types.iteritems():
            print el, t

    def calculate_depth(self, atom):
        depths = {}
        def calc_depth(elem, depth):
            for type in elem.types:
                ex = cgElement(existence, (type,), pddl.TRUE)
                if ex not in depths or depths[ex] > depth+1:
                    depths[ex] = depth+1

            if elem not in self:
                return
            for el2 in self[elem]:
                if el2 not in depths or depths[el2] > depth+1:
                    depths[el2] = depth+1
                    calc_depth(el2, depth+1)

        for elem in self:
            if elem.matches(atom):
                depths[elem] = 0
                calc_depth(elem, 0)

        return depths

def effect_visitor(eff, results):
    if isinstance(eff, pddl.SimpleEffect):
        if eff.predicate in pddl.assignment_ops:
            return [(eff.args[0].function, [a.object for a in eff.args[0].args], eff.args[1].object)]
        if eff.negated:
            return [(eff.predicate, [a.object for a in eff.args], pddl.FALSE)]
        return [(eff.predicate, [a.object for a in eff.args], pddl.TRUE)]
    else :
        return sum(results, [])

def condition_visitor(cond, results):
    if isinstance(cond, pddl.LiteralCondition):
        if cond.predicate == pddl.equals:
            return [(cond.args[0].function, [a.object for a in cond.args[0].args], cond.args[1].object)]
        if cond.negated:
            return [(cond.predicate, [a.object for a in cond.args], pddl.FALSE)]
        return [(cond.predicate, [a.object for a in cond.args], pddl.TRUE)]
    else :
        return sum(results, [])

class Transition(object):
    def __init__(self, function, args, from_val, to_val, conditions=[], effects=[]):
        self.function = function
        self.args = args
        self.from_val = from_val
        self.to_val = to_val
        self.conditions = conditions
        self.coeffects = effects

    def free_vars(self):
        used = set(self.args + [self.to_val])
        if self.from_val:
            used.add(self.from_val)
        result = set()
        for arg in itertools.chain(*(args + [value] for func, args, value in self.conditions)):
            if isinstance(arg, Parameter) and arg not in used:
                result.add(arg)
        return result

    def is_nc_positive(self, func, args, value, cg, only_static = True):
        if isinstance(value, Parameter):
            val = None
        else:
            val = value

        fact = cgElement(func, [a.type for a in args] + [value.type], val)
        if only_static and cg.fact_types[fact] != FACT_TYPE_STATIC:
            return False
        if cg.fact_types[fact] not in (FACT_TYPE_STATIC, FACT_TYPE_FLUENT):
            return False
        return True

    def is_nc_free(self, func, args, value):
        used = set(self.args + [self.to_val])
        if self.from_val:
            used.add(self.from_val)
        if not set(args + [value]) - used:
            return False
        return True

    def complete(self, cg):
        for func, args, value in self.conditions:
            if not self.is_nc_free(func, args, value) and not self.is_nc_positive(func, args, value, cg):
                return False
        return True

    def possibly_complete(self, cg):
        for func, args, value in self.conditions:
#            print "*(%s %s) = %s:  %s/%s" % (func.name, " ".join(str(a) for a in args), str(value), self.is_nc_free(func, args, value), self.is_nc_positive(func, args, value, cg, only_static=False))
            if not self.is_nc_free(func, args, value) and not self.is_nc_positive(func, args, value, cg, only_static=False):
                return False
        return True
    
    def __str__(self):
        return "(%s %s)  %s -> %s" % (self.function.name, " ".join(str(a) for a in self.args), str(self.from_val), self.to_val)
        
            
class DTG(object):
    def __init__(self, function, types, domain, cg):
        self.function = function
        self.types = types
        self.domain = domain
        self.values = set()
        self.transitions = defaultdict(dict)
        self.cg = cg

        self.calculate_transitions()
        self.complete = all(t.complete(cg) for t in self.transitions_iter())
        self.reachable_from = defaultdict(set)
        self.reachable = defaultdict(set)
        self.relevant_transitions = defaultdict(set)
        self.calculate_reachability()
        self.required = defaultdict(set)
        self.calculate_required()

    def transitions_iter(self):
        return itertools.chain(*(d.itervalues() for d in self.transitions.itervalues()))

    def calc_symmetry(self):
        result = True
        for t in self.transitions_iter():
            if isinstance(t.from_val, Parameter) and not isinstance(t.to_val, Parameter) and t.from_val.isInstanceOf(t.to_val.type):
                continue
            if t.from_val == t.to_val:
                continue
            if isinstance(t.from_val, Parameter) and isinstance(t.to_val, Parameter) and t.from_val.type.equalOrSubtypeOf(t.to_val.type):
                continue
            result = False
        return result

    def transitions_for(self, fromval, toval):
        for t in self.transitions_iter():
            if fromval is not None and not abstract_match(fromval, t.from_val):
                    continue
            if toval is not None and not abstract_match(toval, t.to_val):
                    continue
            yield t
    
    def calculate_required(self):
        for t in self.transitions_iter():
            for f, args, val in t.conditions:
                req = self.required[(f, tuple(a.type for a in args))]
                if not any(isinstance(r, pddl.Type) and val.isInstanceOf(r) for r in req):
                    if isinstance(val, Parameter):
                        subtypes = set(r for r in req if isinstance(r, pddl.Type) and r.isSubtypeOf(val.type))
                        req -= subtypes
                        req.add(val.type)
                    else:
                        req.add(val)
        for (f,args), req in self.required.iteritems():
            print "(%s %s) requires: %s" % (f.name, " ".join(a.name for a in args), ", ".join(str(r) for r in req))
                        
    def calculate_reachability(self):
        for v in self.values:
            if isinstance(v, Parameter):
                v = v.type
            if v in self.reachable:
                continue

            open = [v]
            while open:
                value = open.pop()
                for from_val, to_dict in self.transitions.iteritems():
                    matches = False
                    if from_val is None:
                        matches = True
                    elif value is None:
                        matches = False
                    elif isinstance(value, pddl.Type):
                        matches = value.equalOrSubtypeOf(from_val.type)
                    else:
                        if isinstance(from_val, Parameter):
                            matches = value.type.equalOrSubtypeOf(from_val.type)
                        else:
                            matches = (value == from_val)

                    if matches:
                        for to in to_dict.iterkeys():
                            if isinstance(to, Parameter):
                                to = to.type
                    
                            if to not in self.reachable[v]:
                                self.reachable[v].add(to)
                                self.relevant_transitions[v, to] |= set(t for t in to_dict.itervalues())
                                open.append(to)
                                
        for f, to in self.reachable.iteritems():
            for t in to:
                self.reachable_from[t].add(f)
                
        print "reachability for %s:" % self
        for f, to in self.reachable.iteritems():
            print "    %s => %s" % (f, ", ".join(map(str, to)))

        print "reachable from for %s:" % self
        for f, to in self.reachable_from.iteritems():
            print "    %s <= %s" % (f, ", ".join(map(str, to)))
            
        
    def calculate_transitions(self):
        for a in self.domain.actions:
            effects = []
            tr_args = []
            to_vals = []
            for func, args, value in sum([e.visit(effect_visitor) for e in a.effects], []):
                if func == self.function and all(map(lambda arg, t: arg.isInstanceOf(t), args, self.types)):
                    tr_args.append(args)
                    to_vals.append(value)
                else:
                    effects.append((func, args, value))
            if not to_vals:
                continue

            for to_val, to_val_args in zip(to_vals, tr_args):
                conditions = []
                from_val = None
                for func, args, value in a.precondition.visit(condition_visitor):
                    if func == self.function and args == to_val_args:
                        from_val = value
                    else:
                        conditions.append((func, args, value))
                self.values.add(from_val)
                self.values.add(to_val)

                self.transitions[from_val][to_val] = Transition(self.function, to_val_args, from_val, to_val, conditions, effects)

    def __str__(self):
        s =  "(%s %s) - %s" % (self.function.name, " ".join(t.name for t in self.types), self.function.type.name)
        #for t in self.transitions_iter():
        #    s += "\n    %s  nc:%s(%s)" % (t, t.complete(self.cg), t.possibly_complete(self.cg))
        return s

def get_dtgs(func, types, dtgs):
    result = []
    for (f, t), dtg in dtgs.iteritems():
        if func == f and all(map(lambda t1,t2: t1.equalOrSubtypeOf(t2), types, t)):
            result.append(dtg)
    return result
        
def build_dtgs(cg, domain):
    dtgs = {}
    for elem in cg.iterkeys():
        func, types, value = elem
        if (func, types) in dtgs:
            continue
        dtgs[func, types] = DTG(func, types, domain, cg)

    print "\nDTGs:"
    for dtg in dtgs.itervalues():
        print
        print dtg, dtg.calc_symmetry()
        for t in dtg.transitions_iter():
            print "    %s  nc:%s(%s)" % (t, t.complete(cg), t.possibly_complete(cg))
    return dtgs

def print_var(var, mapping):
    argstrs = [mapping[a].pddl_str() for a in var.args]

    s = "(%s %s)" % (var.function.name, " ".join(argstrs))

    if var.modality:
        return "(%s %s)" % (var.modality.name, s)
    return s


def get_required_objects(var, cg):
    if var.modality == pddl.dtpddl.observed:
        return []
            
    #find observations that can lead to this variable
    #effvars = set()
    def forward_edge_func(a, reverse):
        if reverse and not a.rule:
            return False
        if a.conditional or a.rule or a.observation:
            return True

    def expansion_func(node, cg):
        for cond, eff, data in cg.out_edges_iter([node.var], data=True):
            e = data['cgedge']
            if e.conditional or e.rule or e.observation:
                yield eff, e
        for cond, eff, data in cg.in_edges_iter([node.var], data=True):
            e = data['cgedge']
            if e.rule:
                yield eff, e
        
    obs = []
    for v in cg.reachable(var, forward_edge_func):
        if v.modality == pddl.dtpddl.observed:
            obs.append(v)

    if not obs:
        return []
    print "%s can be observed by: %s" % (str(var), ", ".join(str(o) for o in obs))

    #var_instances = cg.reachable_instances(var, forward_edge_func)
    vars, edges = zip(*search_and_map(var, cg, lambda n: n in obs, expansion_func))
    print "involved effects: %s" % (", ".join(str(v) for v in vars))
    print "involved actions: %s" % (", ".join(e.action.name for e in edges if e is not None))
    for cond, edge, eff in zip(vars[:-1], edges[:-1], vars[1:]):
        print "trying to find constraints for %s and %s" % (str(cond), str(eff))
        for c2 in cg.predecessors_iter(eff.var):
            for edata in cg.get_edge_data(c2, eff.var).itervalues():
                e = edata['cgedge']
                if e.action != edge.action:
                    continue
                #find_constraints(cond, edge, eff)
                # for x, ysets in e.graph.edges.iteritems():
                #     for yset, terms in ysets.iteritems():
                #         for t in terms:
                #             print "%s => %s" % (x.pddl_str(), t.pddl_str())
                #for x,t in e.graph.all_bindings():
                #    print "%s => %s" % (x.pddl_str(), t.pddl_str())
                #mnew, newpref = e.bind_vars(c2, eff.var, eff.mapping, eff.preferred)
                #c = VarInstance(c2, mnew)
                #print "  ",c

def find_constraints(var, edge, goal):
    open = set([FunctionTerm(var.var.function, var.var.args)])
    closed = set()
    constraints = defaultdict(list)
    found = None
    while open and not found:
        t = open.pop()
        closed.add(t)
        print t.pddl_str()
        for a, to in edge.graph.all_bindings():
            print a.pddl_str(), to.pddl_str()
            if a == t and to not in closed: # param => term constraint
                constraints[to].append(t)
                if to == goal:
                    found = to
                    break
            elif to == t and a not in closed: # term => param constraint
                constraints[a].append(t)
                if a == goal:
                    found = a
                    break
            elif isinstance(t, pddl.FunctionTerm) and to not in closed: # partial constraints      
                for a2 in t.visit(pddl.predicates.collect_args_visitor):
                    constraints[a2].append(t)
                    constraints[to].append(a2)
                    if to == goal:
                        found = to
                        break
    if not found:
        return []
    
    path = [found]
    next = found
    while constraints[next]:
        next = constraints[next][0]
        path = [next] + path

    print path
    return path
        
    # conds = {}
    # for (v,pref), m in var_instances.iteritems():
    #     for cond in cg.predecessors_iter(v):
    #         for edata in cg.get_edge_data(cond, v).itervalues():
    #             e = edata['cgedge']
    #             mnew, newpref = e.bind_vars(cond, v, m, pref)
    #             #print "%s <= %s" % (print_var(v, m), print_var(cond, mnew))
    #             conds[cond, frozenset(newpref)] = mnew
    # print "involved conditions: %s" % (", ".join(print_var(v,m) for (v,_),m in conds.iteritems()))

class VarInstance(object):
    def __init__(self, var, mapping, free_vars=None):
        self.var = var
        self.mapping = dict((a,mapping[a]) for a in self.var.args if a in mapping)
        self.mapped_args = [mapping.get(a,a) for a in self.var.args]
        self.hash = hash(tuple([self.var] + self.mapped_args))

        if free_vars is None:
            self.free = set(sum((a.visit(pddl.predicates.collect_args_visitor) for a in self.mapped_args), []))
        else:
            self.free = free_vars

        self.preferred = set(k for k in self.mapping.iterkeys() if k not in self.free)
        #print self
        #print "*",["%s => %s" % (k.pddl_str(), v.pddl_str()) for k,v in self.mapping.iteritems()]
        #print "*",[p.pddl_str() for p in self.preferred]

    def __hash__(self):
        return self.hash
    
    def __eq__(self, o):
        return self.__class__ == o.__class__ and self.hash == o.hash

    def __str__(self):
        argstrs = [a.pddl_str() for a in self.mapped_args]

        s = "(%s %s)" % (self.var.function.name, " ".join(argstrs))

        if self.var.modality:
            return "(%s %s)" % (self.var.modality.name, s)
        return s
        
    
def search_and_map(var, cg, target_func=lambda n: False, expansion_func=None):
    def default_expansion(node, cg):
        for cond, eff, data in cg.out_edges_iter([next.var], data=True):
            e = data['cgedge']
            yield eff, e
    if expansion_func is None:
        expansion_func = default_expansion

    init = VarInstance(var, {})
    open = set([init])
    closed = {init : None}

    found = None
    if target_func(var):
        found = init
        
    while open and not found:
        next = open.pop()
        for succ, edge in expansion_func(next, cg):
            mnew, newpref = edge.bind_vars(succ, next.var, next.mapping, next.preferred)
            succ_i = VarInstance(succ, mnew)
            #print "%s => %s" % (str(next), str(eff_i))
            if succ_i in closed:
                continue
            open.add(succ_i)
            closed[succ_i] = (next, edge)
            
            if target_func(succ):
                found = succ_i
                break

    if not found:
        return []
    
    path = [(found,None)]
    next = found
    while closed[next] is not None:
        next, edge = closed[next]
        path = [(next, edge)] + path
                
    return path
        
if __name__ == '__main__':    
    assert len(sys.argv) == 3, """Call 'planner.py domain.mapl task.mapl' for a single planner call"""
    domain_fn, problem_fn = sys.argv[1:]
    domain = pddl.load_domain(domain_fn)

    t = pddl.translators.ObjectFluentNormalizer()
    dom = t.translate(domain)
    dom.init_rules = [t.translate_action(r) for r in domain.init_rules]
    print ("\n".join(pddl.writer.Writer().write_domain(dom)))
    for r in dom.init_rules:
        print ("\n".join(pddl.writer.Writer().write_action(r)))
    cg = CausalGraph(dom)

    exvar = AbstractVariable(dom.predicates["ex-in-room"][0], [Term(Parameter("?l", dom.types["label"])), Term(Parameter("?r", dom.types["room"]))])
    print map(str, cg.get_probabilistic_vars([exvar]))
    for var in cg.nodes_iter():
        get_required_objects(var, cg)
        # if var.modality == pddl.dtpddl.observed:
        #     actions = set()
        #     def edge_func(a, reverse):
        #         if reverse:
        #             return False
        #         if a.conditional or a.rule or a.observation:
        #             actions.add(a.action)
        #             return True
        #     print str(var) + ": " + ", ".join(map(str, cg.reachable_from(var, edge_func)))
        #     print [a.name for a in actions]
        #     print str(var) + ": " + ", ".join(print_var(v,m) for (v,_),m in cg.reachable_from_instance(var, edge_func).iteritems())
