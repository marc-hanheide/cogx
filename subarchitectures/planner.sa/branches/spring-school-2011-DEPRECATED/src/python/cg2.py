import itertools
import networkx

from collections import defaultdict


from standalone import pddl
from standalone.pddl import visitors


class CGNode(object):
    def __init__(self, function, args):
        self.function = function
        self.args = args
        self.hash = hash(self.signature())
        
    @staticmethod
    def from_atom(atom):
        if atom.predicate in [pddl.equals] + pddl.assignment_ops + pddl.numeric_ops + pddl.builtin.numeric_comparators:
            func = atom.args[0].function
            args = atom.args[0].args
        else:
           func = atom.predicate
           args = atom.args
        return CGNode(func, args)

    def as_term(self):
        return pddl.FunctionTerm(self.function, self.args)

    def signature(self):
        sig = [self.function]
        for a in self.args:
            if isinstance(a, pddl.VariableTerm):
                sig.append(a.get_type())
            elif isinstance(a, pddl.ConstantTerm):
                sig.append(a.object)
            else:
                assert False
        return tuple(sig)

    def really_equal(self, other):
        return self.function == other.function and all(a == o for a,o in zip(self.args, other.args))
    
    def subsumed_by(self, other):
        if self.function != other.function:
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
        return "(%s %s)" % (self.function.name, " ".join(str(a.object.name) for a in self.args))
    
    def __hash__(self):
        return self.hash
    
    def __eq__(self, other):
        return isinstance(other, type(self)) and other.hash == self.hash

def graph_iterator(d, edge_filter=lambda x: bool(x)):
    for k1, k2dict in d.iteritems():
        for k2, val in k2dict.iteritems():
            if edge_filter(val):
                yield k1, k2, val
    
class ConstraintGraph(object):
    def __init__(self, action):
        self.action = action
        self.compute()

    def extract_simple_constraints(self):
        equalities = []
        nonequalities = []
        
        def condition_visitor(cond, results):
            if isinstance(cond, pddl.LiteralCondition):
                if cond.predicate in (pddl.equals, pddl.builtin.eq):
                    assert isinstance(cond.args[0], pddl.FunctionTerm) and isinstance(cond.args[1], (pddl.ConstantTerm, pddl.VariableTerm))
                    if cond.negated:
                        nonequalities.append((cond.args[0], cond.args[1]))
                    else:
                        equalities.append((cond.args[0], cond.args[1]))
            
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.ConditionalEffect):
                eff.condition.visit(condition_visitor)
                
        visitors.visit(self.action.precondition, condition_visitor)
        visitors.visit(self.action.effect, effect_visitor)

        return equalities, nonequalities

    def compute(self):
        eq, noneq = self.extract_simple_constraints()
        graph = defaultdict(lambda: defaultdict(lambda: False))
        closed = defaultdict(set)
        
        for arg in self.action.args:
            t = pddl.Term(arg)
            graph[t][t] = True
        for e1,e2 in eq:
            graph[e1][e2] = True
            graph[e2][e1] = True
            
        done = False
        while not done:
            done = True
            new = []
            for e1, e2,_ in graph_iterator(graph):
                for e3 in graph[e2].iterkeys():
                    if e3 not in graph[e1]:
                        new.append((e1,e3))
                        done = False
                if isinstance(e2, pddl.FunctionTerm):
                    for i, arg in enumerate(e2.args):
                        for e3 in graph.get(arg,{}).iterkeys():
                            if e3 == e1:
                                continue # avoid (some) cycles
                            t = pddl.FunctionTerm(e2.function, e2.args[:i] + [e3] + e2.args[i+1:])
                            if t not in graph[e1]:
                                new.append((e1, t))
                                done = False
                        
            for e1,e2 in new:
                graph[e1][e2] = True
                graph[e2][e1] = True

        self.graph = graph
        print "====",self.action.name
        for e1, e2, _ in graph_iterator(graph):
            print "%s => %s" % (e1.pddl_str(), e2.pddl_str())

    def replace(self, args, allowed):
        newargs = []
        allowed = set(a.object for a in allowed)
        for arg in args:
            found = False
            for rep in self.graph[arg].iterkeys():
                if not (set(rep.visit(pddl.predicates.collect_args_visitor)) - allowed):
                    newargs.append(rep)
                    found = True
                    break
            if not found:
                newargs.append(arg)
        return newargs

class CGNodeMapping(object):
    def __init__(self, node, term, value):
        self.node = node
        self.term = term
        self.value = value
        self.to_node_map = dict((k,v) for k,v in zip(term.args, node.args))
        self.from_node_map = dict((k,v) for k,v in zip(node.args, term.args))

    def set_node(self, node):
        self.node = node
        self.to_node_map = dict((k,v) for k,v in zip(self.term.args, node.args))
        self.from_node_map = dict((k,v) for k,v in zip(node.args, self.term.args))
        
    @staticmethod
    def from_atom(atom):
        if atom.predicate in [pddl.equals] + pddl.assignment_ops + pddl.numeric_ops + pddl.builtin.numeric_comparators:
            term = atom.args[0]
            val = atom.args[1]
        else:
            #TODO: make this more rigorous
            term = pddl.FunctionTerm(atom.predicate, atom.args)
            if atom.negated:
                val = pddl.FALSE
            else:
                val = pddl.TRUE
        avar = CGNode.from_atom(atom)
        return CGNodeMapping(avar, term, val)
        
class CGEdge(object):
    def __init__(self, cond, eff, action, probabilistic = False, conditional = False  ):
        self.cnode = cond
        self.enode = eff
        self.action = action

        print action.name, cond.node, eff.node, eff.term.pddl_str()
        
        self.constraints = ConstraintGraph(action)        

        self.probabilistic = probabilistic
        self.conditional = conditional
        self.observation = isinstance(self.action, pddl.dtpddl.Observation)
        self.rule = isinstance(self.action, pddl.mapl.InitRule)

    def term_to_node_repr(self, term):
        if term == self.cnode.term:
            t = pddl.FunctionTerm(term.function, [self.cnode.to_node_map[a] for a in term.args])
        elif term == self.enode.term:
            t = pddl.FunctionTerm(term.function, [self.enode.to_node_map[a] for a in term.args])
        else:
            assert False
        return t

    def term_from_node_repr(self, term):
        if term.function == self.cnode.function:
            t = pddl.FunctionTerm(term.function, [self.cnode.from_node_map[a] for a in term.args])
        elif term.function == self.enode.function:
            t = pddl.FunctionTerm(term.function, [self.enode.from_node_map[a] for a in term.args])
        else:
            assert False
        return t

    def bind(self, node1, node2, mapping=None, preferred=None):
        #node => node-mapping
        for n in (self.enode, self.cnode):
            if node1 == n.node:
                node1 = n
            elif node2 == n.node:
                node2 = n
                
        if mapping is None:
            mapping = dict((a,a) for a in node1.term.args)
        else:
            mapping = dict((a, mapping[node1.to_node_map[a]]) for a in node1.term.args)

        if preferred is None:
            preferred = set(mapping.keys())
        else:
            preferred = set(node1.from_node_map[a] for a in preferred)
        
        #t = self.term_from_node_repr(var1.term)
        repl = self.constraints.replace(node2.term.args, preferred)
        assert repl

        t = pddl.FunctionTerm(node2.term.function, repl)
        def mapping_replace_visitor(term, results):
            if isinstance(term, pddl.FunctionTerm):
                return pddl.FunctionTerm(term.function, results)
            return mapping.get(term, term)
        t2 = t.visit(mapping_replace_visitor)

        result = dict((node2.to_node_map[a], b) for a,b in zip(node2.term.args, t2.args))
        return result

    def bind_term(self, term, node1, node2):
        mapping = dict((a,t) for a,t in zip(node1.args, term.args))
        mnew = self.bind(node1, node2, mapping)
        return pddl.FunctionTerm(node2.function, [mnew[a] for a in node2.args])
        
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
        self.domain = domain
        self.build_cg(self.domain)

    def build_cg(self, domain):
        cg = self

        @visitors.collect
        def effect_visitor(eff, results):
            if isinstance(eff, pddl.SimpleEffect):
                return ([], False, CGNodeMapping.from_atom(eff))
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
                return CGNodeMapping.from_atom(cond)

        actions = domain.actions + domain.init_rules + domain.observe

        canonical_nodes = {}
            
        for a in actions:
            conds = set(visitors.visit(a.precondition, condition_visitor, []))
            for econds, prob, eff in visitors.visit(a.effect, effect_visitor, []):
                eff.set_node(canonical_nodes.setdefault(eff.node,eff.node))
                for c in conds | set(econds):
                    if c.term == eff.term:
                        continue
                    c.set_node(canonical_nodes.setdefault(c.node,c.node))
                    c_eff = c in econds
                    cg.add_edge(c.node, eff.node, cgedge=CGEdge(c, eff, a, probabilistic=prob, conditional=c_eff) )

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

    def reachable(self, fromvar, expansion_func=None):
        def default_expansion(node, cg):
            for cond, eff, data in cg.out_edges_iter([node], data=True):
                e = data['cgedge']
                yield eff, e
        if expansion_func is None:
            expansion_func = default_expansion

        start_in_cycle = False
        open = set([fromvar])
        closed = set()
        while open:
            next = open.pop()
            closed.add(next)
            for succ, edge in expansion_func(next, self):
                if succ == fromvar:
                    start_in_cycle = True
                if succ in closed:
                    continue
                open.add(succ)
                
        if not start_in_cycle:
            closed.discard(fromvar)
        return closed

    def reachable_instances(self, fromvar, expansion_func=None):
        def default_expansion(node, cg):
            for cond, eff, data in cg.out_edges_iter([node], data=True):
                e = data['cgedge']
                yield eff, e
        if expansion_func is None:
            expansion_func = default_expansion

        init_term = fromvar.as_term()
        nodedict = {}
        nodedict[init_term] = fromvar
        start_in_cycle = False
        open = set([init_term])
        closed = set()
        while open:
            next = open.pop()
            nextnode = nodedict[next]
            closed.add(next)
            for succ, edge in expansion_func(nextnode, self):
                sterm = edge.bind_term(next, nextnode, succ)
                print next.pddl_str(), "=>", sterm.pddl_str()
                nodedict[sterm] = succ
                if sterm == init_term:
                    start_in_cycle = True
                if sterm in closed:
                    continue
                open.add(sterm)
                
        if not start_in_cycle:
            closed.discard(init_term)
        return closed
    
def forward_search(func=lambda x:True):
    def successors(node, cg):
        for cond, eff, data in cg.out_edges_iter([node], data=True):
            e = data['cgedge']
            if func(e):
                yield eff, e
    return successors

def backward_search(func=lambda x:True):
    def successors(node, cg):
        for cond, eff, data in cg.in_edges_iter([node], data=True):
            e = data['cgedge']
            if func(e):
                yield cond, e
    return successors

def chain_search(*funcs):
    def chain_successors(node, cg):
        for f in funcs:
            for x in f(node, cg):
                yield x

    return chain_successors

def bidir_search(fwd=lambda x:True, backwd=None):
    if backwd is None:
        backwd = fwd
    return chain_search(forward_search(fwd), backward_search(backwd))

def get_prob_vars(cg, prob_vars=[]):
    #1. find all probabilistic effects
    pfunc = forward_search(lambda e: e.probabilistic)
    result = set()
    for var in cg.nodes_iter():
        result |= cg.reachable(var, pfunc)
    #for var in prob_vars:
    #    result |= cg.reachable(var, forward_search())
    return result

def get_observable_vars(cg):
    func = bidir_search(lambda e: e.rule, lambda e: e.conditional or e.rule or e.observation)
    result = set()
    for var in cg.nodes_iter():
        if var.function.name.startswith("observed-"):
            result |= cg.reachable_instances(var, func)
    return result

if __name__ == '__main__':    
    import sys
    
    assert len(sys.argv) == 3, """Call 'planner.py domain.mapl task.mapl' for a single planner call"""
    domain_fn, problem_fn = sys.argv[1:]
    domain = pddl.load_domain(domain_fn)

    t = pddl.translators.ObjectFluentNormalizer()
    t2 = pddl.translators.ModalPredicateCompiler()
    dom = t2.translate(t.translate(domain))
    dom.init_rules = [t2.translate_action(t.translate_action(r), dom) for r in domain.init_rules]
    #print ("\n".join(pddl.writer.Writer().write_domain(dom)))
    #for r in dom.init_rules:
    #    print ("\n".join(pddl.writer.Writer().write_action(r)))
    cg = CausalGraph(dom)
    print map(str, get_prob_vars(cg))
    print [term.pddl_str() for term in get_observable_vars(cg)]
