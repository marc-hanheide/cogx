import time
from itertools import chain
from collections import defaultdict

import pddl
from pddl import state, effects, dtpddl, mapl

class PNode(object):
    nodeid = 0
    actionid = 0
        
    def __init__(self, svar, children):
        self.svar = svar
        self.parent = None
        self._children = children
        self.is_unified = False
        self.reachable_dict = {}
        
        for p, nodes, facts in self._children.itervalues():
            for n in nodes:
                n.parent = self
        
        self.branch_vars = self.get_branch_vars()
        self.normalize()

        #self.hash = hash(tuple(self.children)) #TODO make order invariant

    def is_expanded(self):
        return True

    def get_children(self):
        if not self.is_unified:
            self.unify_branches()
        return self._children
        
    children = property(get_children)
    
    @staticmethod
    def from_effect(peff, rules=[], stat=None):
        def get_children(eff):
            if isinstance(eff, pddl.ConjunctiveEffect):
                return eff.parts
            else:
                return [eff]

        def get_firing_rules(facts):
            for r in rules:
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in r.deps:
                        rule_facts[svar] = v
                if rule_facts:
                    yield (r, rule_facts)
            
        obj = pddl.TypedObject("node%d" % PNode.nodeid, dtpddl.t_node)
        PNode.nodeid += 1
        svar = state.StateVariable(dtpddl.selected, [obj])
            
        children = {}
        p_total = 0
        for i, (p, eff) in enumerate(peff.get_effects(None)):
            # p = p.object.value
            facts = [state.Fact.from_literal(e) for e in get_children(eff) if isinstance(e, pddl.SimpleEffect)]
            facts = dict((f.svar, f.value) for f in facts)
            nodes = [PNode.from_effect(e, rules, stat) for e in get_children(eff) if isinstance(e, effects.ProbabilisticEffect)]
            for rule, fixed in get_firing_rules(facts):
                nodes += LazyPNode.from_rule(rule, rules, fixed, stat)
            val = pddl.TypedObject("choice%d" % i, dtpddl.t_node_choice)
                
            children[val] = (p, nodes, facts)
            p_total += p

        assert p_total <= 1.0001, "p(%s) = %.4f (%s)" % (str(svar), p_total, peff.pddl_str())
        return PNode(svar, children)


    @staticmethod
    def from_fact(fact, rules=[], stat=None):
        def get_firing_rules(facts):
            for r in rules:
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in r.deps:
                        rule_facts[svar] = v
                if rule_facts:
                    yield (r, rule_facts)
            
        svar = fact.svar
        children = {}
        p_total = 0
        for i, (val, p) in enumerate(fact.value.iteritems()):
            facts = {svar : val}
            nodes = []
            for rule, fixed in get_firing_rules(facts):
                # print rule, map(str, fixed)
                nodes += LazyPNode.from_rule(rule, rules, fixed, stat)
                
            children[val] = (p, nodes, facts)
            p_total += p

        if p_total > 1.0:
            for val, (p, nodes, facts) in children.iteritems():
                children[val] = (p/p_total, nodes, facts)

        # assert p_total <= 1.0001, "p(%s) = %.4f (%s)" % (str(svar), p_total, peff.pddl_str())
        return PNode(svar, children)
    
    def hash(self):
        hashes = []
        for val, (p, nodes, facts) in self._children.iteritems():
            hashes.append(hash((p, frozenset([n.hash() for n in nodes]), frozenset(facts.iteritems()))))
        return hash(frozenset(hashes))
    
    def check_node(self, facts, stat, qgraph, eval_fn=None, parent_facts={}):
        # print "check:", self, map(str, facts)
        def sub_eval_fn(it):
            # print "returning iterator"
            return it
        def check_fn(facts):
            fset = set(facts)
            # print "facts:", map(str, fset)
            for val, (p, nodes, nfacts) in self.children.iteritems():
                new_parent_facts = parent_facts.copy()
                factgen = (state.Fact(s,v) for s,v in chain(nfacts.iteritems(), [state.Fact(self.svar, val,)]))
                for f in factgen:
                    # print "..",f
                    if f in fset:
                        # print "yeaH!"
                        fset.discard(f)
                        yield True, f
                    new_parent_facts[f.svar] = f.value
                for n in nodes:
                    # print "descend:", n
                    for res, f in n.check_node([f for f in facts if f.svar not in parent_facts], stat, qgraph, sub_eval_fn, new_parent_facts):
                        # print "from children", f, res
                        if res:
                            fset.discard(f)
                        yield res, f
            for f in fset:
                yield False, f

        def fact_iter(facts):
            # print "  start iterating:", map(str, facts)
            remaining = []
            for f in facts:
                # print "   test:", f
                if f in self.reachable_dict:
                    # print "    cached", f, self.reachable_dict[f]
                    yield self.reachable_dict[f], f
                else:
                    remaining.append(f)
            if remaining:
                # print "   not cached:", map(str, remaining)
                for res, f in check_fn(remaining):
                    prev_res = self.reachable_dict.get(f, None)
                    if prev_res is None or (prev_res is False and res is True):
                        self.reachable_dict[f] = res
                        # print self, f, res
                        yield res, f
            
        if eval_fn is None:
            # def anyfn(x):
            #     for res, f in x:
            #         print "  ", res, f
            #         if res:
            #             return True
            #     return False
            eval_fn = lambda x: any(res for res, f in x)
            # eval_fn = anyfn
        # print "start eval", map(str, facts)

        return eval_fn(fact_iter(facts))

    @staticmethod
    def unify_nodes(nodes):
        def join_nodes(svar, nodes):
            children = {}
            for n in nodes:
                for val, (p, nds, facts) in n.children.iteritems():
                    if val in children:
                        p2, nds2, facts2 = children[val]
                        facts = facts.copy()
                        facts.update(facts2)
                        children[val] = (p+p2, nds+nds2, facts)
                    else:
                        children[val]= (p, nds, facts)
            return PNode(svar, children)
        
        nodedict = defaultdict(list)
        for n in nodes:
            nodedict[n.svar].append(n)

        for svar, nds in nodedict.iteritems():
            # print svar, len(nds)
            if len(nds) == 1:
                yield nds[0]
            else:
                yield join_nodes(svar, nds)

    def normalize(self):
        p_sum = sum(p for p, _, _ in self.children.itervalues())
        if p_sum <= 1.0:
            return
        norm = 1.0 / p_sum
        for val, (p, nodes, facts) in self.children.iteritems():
            self.children[val] = (p*norm, nodes, facts)

    def unify_branches(self):
        """if a node contains several children with the same svar,
        join those into one node. This changes the semantics of the
        node, but is usually the right thing to do, because unifying
        nodes during rule expansion is harder to do."""

        # def join_nodes(svar, nodes):
        #     children = {}
        #     for n in nodes:
        #         for val, (p, nds, facts) in n.children.iteritems():
        #             if val in children:
        #                 p2, nds2, facts2 = children[val]
        #                 facts = facts.copy()
        #                 facts.update(facts2)
        #                 children[val] = (p+p2, nds+nds2, facts)
        #             else:
        #                 children[val]= (p, nds, facts)
        #     return PNode(svar, children)

        # print "unify:", self

        for p, nodes, facts in self._children.itervalues():
            new_nodes =  []
            for n in PNode.unify_nodes(nodes):
                n.parent = self
                new_nodes.append(n)
            nodes[:] = new_nodes
            # nodedict = defaultdict(list)
            # for n in nodes:
            #     nodedict[n.svar].append(n)

            # nodes[:] = []
            # for svar, nds in nodedict.iteritems():
            #     # print svar, len(nds)
            #     if len(nds) == 1:
            #         nodes += nds
            #     else:
            #         newnode = join_nodes(svar, nds)
            #         newnode.parent = self
            #         nodes.append(newnode)

    def consolidate(self, nodedict):
        for val, (p, nodes, facts) in self._children.iteritems():
            nodes = [n.consolidate(nodedict) for n in nodes]
            self._children[val] = (p, nodes, facts)
        
        if self.hash() in nodedict:
            #print "duplicate found"
            return nodedict[self.hash()]
        nodedict[self.hash()] = self
        return self

    def replace_branch_var(self, svar):
        self.svar = svar
        children_new = {}
        for val, (p, nodes, facts) in self._children.iteritems():
            newval = facts[svar]
            del facts[svar]
            children_new[newval] = (p, nodes, facts)
        self._children = children_new

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
        if self.svar in ndict:
            return ndict, fdict
        # assert ndict.get(self.svar, self) == self, self.svar
        
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
        noderesult = list(PNode.unify_nodes(noderesult))
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
                cleaned_facts = dict((svar, val) for svar, val in facts.iteritems() if svar.function != dtpddl.selected)
                return nodes, cleaned_facts, True
            
        return [], {}, False


    def size(self, selected_facts=None):
        # if selected_facts is not None and self.svar not in selected_facts:
        #     print "tree not evaluated:", self.svar
        #     return 1
        print "start:", self.svar
        size = 0
        total_p = 0
        for val, (p, nodes, facts) in self.children.iteritems():
            total_p += p
            # if selected_facts is not None and val not in selected_facts[self.svar]:
            #     print "branch not evaluated:", self.svar, val
            #     continue
            bsize = 1
            for n in nodes:
                # print "child", n.svar
                bsize *= n.size(selected_facts)
            size += bsize
            print "evaluated:", self.svar, val, bsize
        if total_p < 1.0:
            size += 1
        print "size of", self.svar, size
        return size

    def add_facts(self, choices):
        cval = choices.get(self.svar, None)
        if cval and cval in self.children:
            p, nodes, facts = self.children[cval]
            for n in nodes:
                #print "subtrees of %s=%s" % (str(self.svar), str(cval));
                yield n.all_facts()
        for val, (p, nodes, facts) in self.children.iteritems():
            if val == cval:
                continue
            result = set(state.Fact(svar, val) for svar, val in facts.iteritems())
            #result.add(state.Fact(self.svar, val))
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
            #result.add(state.Fact(self.svar, val))
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
        s = state.State([], problem)
        peffs = []
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                peffs.append(i)
            else:
                s.set(state.Fact.from_literal(i))

        try:
            rules = problem.domain.dt_rules
        except:
            rules = []
            
        nodes = []
        for e in peffs:
            nodes.append(PNode.from_effect(e, rules, s))
            
        return nodes

    @staticmethod
    def from_state(probstate, detstate):
        try:
            rules = probstate.problem.domain.dt_rules
        except:
            rules = []
            
        nodes = []
        for r in rules:
            # fire all rules depending only on deterministic facts
            nodes += LazyPNode.from_rule(r, rules, {}, detstate)
            
        for fact in probstate.iterdists():
            if fact.value.value is None:
                nodes.append(PNode.from_fact(fact, rules, detstate))
            
        return nodes
    
    def prepare_actions(self):
        pass
        # self.visited_by = set()
        # for val, (p, nodes, facts) in self.children.iteritems():
        #     for n in nodes:
        #         n.prepare_actions()

    def to_actions(self, domain, parent_conds=None, parent_p=1.0, filter_func=None):
        actions = []
        # if parent_conds:
        #     if frozenset(parent_conds) in self.visited_by:
        #         return []
        #     self.visited_by.add(frozenset(parent_conds))

        for o in self.svar.args:
            domain.add_constant(o)
            
        started_pred = domain.predicates.get("started", [])
            
        for val, (p, nodes, facts) in self.children.iteritems():
            if p * parent_p <= 0.001 or val == pddl.UNKNOWN:
                # print "pruned due to likelihood"
                continue
            
            use_this = use_children = True
            if filter_func:
                use_this, use_children = filter_func(self, val, p ,facts)
                if not use_this and not use_children:
                    continue
                                               
            name = "__commit-%s-%s-%s" % (self.svar.function.name, "-".join(a.name for a in self.svar.args), val.name)
            PNode.actionid += 1
            domain.add_constant(val)
                
            agent = pddl.Parameter("?a", mapl.t_planning_agent)
            a = mapl.MAPLAction(name, [agent], [], [], pddl.Conjunction([]), None, pddl.ConjunctiveEffect([]), [], domain)
            if use_this:
                actions.append(a)
            b = pddl.Builder(a)
            
            if started_pred:
                a.precondition.parts.append(b.cond("not", (started_pred, )))
                
            a.precondition.parts.append(b.cond("not", ("committed", self.svar.as_term())))
            # a.precondition.parts.append(b.cond("not", ("started",)))
            if parent_conds:
                a.precondition.parts += [c.copy(a) for c in parent_conds]

            for svar, value in chain(facts.iteritems(), [(self.svar, val)]):
                for c in chain(svar.args, [value]):
                    domain.add_constant(c)
                cvar = svar.as_modality(mapl.commit, [value])
                a.effect.parts.append(cvar.as_literal(_class=effects.SimpleEffect))
                # a.precondition.parts.append(b.cond("not", ("committed", svar.as_term())))
                
            a.effect.parts.append(b.effect("assign", ("probability",), p))
            a.set_total_cost(0)

            new_parent_cond = b.cond("=", self.svar.as_term(), val)

            child_actions = []
            for n in nodes:
                child_actions += n.to_actions(domain, [new_parent_cond], p * parent_p, filter_func)

            if child_actions and use_children:
                if not use_this:
                    actions.append(a)
                actions += child_actions

        return actions

    def to_init(self, selected_facts=None, filter_fn=None, parent_facts={}):
        def make_unknown(val):
            return pddl.TypedObject("other-%s" % val.type.name, val.type)

        written_facts = set()
        if filter_fn and not filter_fn(self, parent_facts):
            return None, written_facts

        t0 = time.time()
        
        selected_svars = set(f.svar for f in selected_facts) if selected_facts is not None else None

        effs = []
        normalize = 1.0
        p_sum = sum(p for p,_,_ in self.children.itervalues())
        if p_sum > 1.0:
            normalize = 1.0 / p_sum
        
        for val, (p, nodes, facts) in self.children.iteritems():
            if p < 0.0001:
                continue
            ceff = pddl.ConjunctiveEffect([])
            new_parent_facts = parent_facts.copy()
            for svar, val in chain(facts.iteritems(), [(self.svar, val)]):
                new_parent_facts[svar] = val
                #if svar.function == selected:
                #    continue
                f = state.Fact(svar, val)
                if selected_facts is not None and f not in selected_facts:
                    # if f.svar in selected_svars and f.svar.function != selected:
                    #     f = state.Fact(svar, make_unknown(val))
                    # else:
                    continue
                written_facts.add(f)
                    
                eff = f.as_literal(_class=pddl.SimpleEffect)
                ceff.parts.append(eff)
            for n in nodes:
                eff, new_written = n.to_init(selected_facts, filter_fn, new_parent_facts)
                if eff:
                    ceff.parts.append(eff)
                written_facts |= new_written
            if ceff.parts:
                effs.append((pddl.Term(p*normalize), ceff))
        # print "%s took: %.2f secs" % (self, time.time()-t0)
        if effs:
            return effects.ProbabilisticEffect(effs), written_facts
        return None, set()

    # def to_init(self, selected_facts=None, observable_facts=None, filter_fn=None, parent_facts={}):
    #     def make_unknown(val):
    #         return pddl.TypedObject("other-%s" % val.type.name, val.type)

    #     if filter_fn and not filter_fn(self, parent_facts):
    #         return None, False

    #     t0 = time.time()
        
    #     selected_svars = set(f.svar for f in selected_facts) if selected_facts is not None else None
    #     observable = False if observable_facts is not None else True
    #     effs = []
    #     normalize = 1.0
    #     p_sum = sum(p for p,_,_ in self.children.itervalues())
    #     if p_sum > 1.0:
    #         normalize = 1.0 / p_sum
        
    #     for val, (p, nodes, facts) in self.children.iteritems():
    #         if p < 0.0001:
    #             continue
    #         ceff = pddl.ConjunctiveEffect([])
    #         new_parent_facts = parent_facts.copy()
    #         for svar, val in chain(facts.iteritems(), [(self.svar, val)]):
    #             new_parent_facts[svar] = val
    #             #if svar.function == selected:
    #             #    continue
    #             f = state.Fact(svar, val)
    #             if selected_facts is not None and f not in selected_facts:
    #                 # if f.svar in selected_svars and f.svar.function != selected:
    #                 #     f = state.Fact(svar, make_unknown(val))
    #                 # else:
    #                 continue
    #             if observable_facts is not None and f in observable_facts:
    #                 observable = True
                    
    #             eff = f.as_literal(_class=pddl.SimpleEffect)
    #             ceff.parts.append(eff)
    #         for n in nodes:
    #             eff, obs = n.to_init(selected_facts, observable_facts, filter_fn, new_parent_facts)
    #             if eff:
    #                 ceff.parts.append(eff)
    #             if obs:
    #                 observable = True
    #         if ceff.parts:
    #             effs.append((pddl.Term(p*normalize), ceff))
    #     # print "%s took: %.2f secs" % (self, time.time()-t0)
    #     if effs and observable:
    #         return effects.ProbabilisticEffect(effs), observable
    #     return None, False

    def __str__(self):
        return "PNode: %s (%d: %s)" % (str(self.svar), len(self.children), ", ".join(v.name for v in self.children.iterkeys()) )

        
class LazyPNode(PNode):
    def __init__(self, rule, mapping, branch, all_rules):
        self.rule = rule
        self.all_rules = all_rules
        self.mapping = mapping
        self.parent = None
        self._children = None
        self.svar = branch
        self.reachable_dict = {}

    def simplify(self):
        return [], {}, False

    def is_expanded(self):
        return self._children is not None
        
    def get_children(self):
        if self._children is not None:
            return self._children

        self._children = {}
        # print ["%s = %s" % (str(k),str(v)) for k,v in self.mapping.iteritems() ]

        def get_firing_rules(facts):
            for r in self.all_rules:
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in r.deps:
                        rule_facts[svar] = v
                if rule_facts:
                    yield (r, rule_facts)
        
        def get_objects(arg):
            return list(self.state.problem.get_all_objects(arg.type))

        # print "state:"
        # print self.state
        args = self.rule.args
        # print "creating subtree for %s using rule %s" % (str(self.svar), self.rule.name)
        # print ["%s = %s" % (str(k),str(v)) for k,v in self.mapping.iteritems() ]
        for mapping in self.rule.smart_instantiate(self.rule.get_inst_func(self.state), args, [get_objects(a) for a in args], self.state.problem, self.mapping):
            # log.debug("creating subtree for %s", str(self.svar))
            
            # print "  ", ["%s = %s" % (str(k),str(v)) for k,v in mapping.iteritems() ]
            for p, values in self.rule.values:
                p = self.state.evaluate_term(p)
                if p == pddl.UNKNOWN or p.value < 0.01:
                    continue
                
                facts = {}
                for (func, fargs), val in zip(self.rule.variables, values):
                    svar = state.StateVariable(func, state.instantiate_args(fargs))
                    val = self.state.evaluate_term(val)
                    if svar in self.state:
                        print svar, "already defined."
                        facts = None
                        break
                    facts[svar] = val
                    
                if not facts:
                    continue
                # print "  generating child for %s = %s (p=%.2f)" % (str(self.svar), str(val), p.value)

                nodes = []
                # for r, f in get_firing_rules(facts):
                #     print r
                #     print ["%s = %s" % (str(k), str(v)) for k,v in facts.iteritems()]

                for rule, fixed in get_firing_rules(facts):
                    nodes += LazyPNode.from_rule(rule, self.all_rules, fixed, self.state)

                nodeval = facts[self.svar]

                self._children[nodeval] = (p.value, nodes, facts)

        self.unify_branches()
        return self._children
            
    children = property(get_children)

    
    def check_node(self, facts, state, qgraph, eval_fn=None, parent_facts={}):
        # print "check:", self
        def state_query_fn(svar):
            return parent_facts.get(svar, state[svar])
        def check_fn(facts):
            rule = self.rule
            args = [self.mapping.get(a,a) for a in rule.args]
            for f in facts:
                # print "|",["%s=%s" % x for x in parent_facts.iteritems()]
                yield qgraph.query_reachability(rule, args, f, state.problem, state_query_fn), f

        def fact_iter(facts):
            remaining = []
            for f in facts:
                if f in self.reachable_dict:
                    yield self.reachable_dict[f], f
                else:
                    remaining.append(f)
            if remaining:
                for res, f in check_fn(remaining):
                    self.reachable_dict[f] = res
                    # print self, map(str, f), res
                    yield res, f
            
        if eval_fn is None:
            eval_fn = lambda x: any(res for res, f in x)

        return eval_fn(fact_iter(facts))

    @staticmethod
    def from_rule(rule, rules, fixed_facts, stat):
        stat = state.SubState(stat)
        for svar, val in fixed_facts.iteritems():
            stat[svar] = val

        #print "\nfrom rule:", rule
            
        pre_mapping = rule.match_args(state.Fact(svar,val) for svar, val in fixed_facts.iteritems())
        # print "mapping", ["%s = %s" % (str(k),str(v)) for k,v in pre_mapping.iteritems() ]
        
        if fixed_facts and not pre_mapping:
            #rule doesn't match at all
            return []
        # print "initial rule", ["%s = %s" % (str(k),str(v)) for k,v in pre_mapping.iteritems() ]

        # def get_cached_rule(rule, mapping, svar):
        #     try:
        #         cache = rule.pnode_cache
        #     except:
        #         rule.pnode_cache = {}
        #     return rule.pnode_cache.setde

        def get_objects(arg):
            return list(stat.problem.get_all_objects(arg.type))

        # for a in rule.args:
        #     print a, map(str, get_objects(a))
        nodes = []
        for mapping in rule.smart_instantiate(rule.get_inst_func(stat), rule.args, [get_objects(a) for a in rule.args], stat.problem, pre_mapping):
            func, fargs = rule.variables[0]
            svar = state.StateVariable(func, state.instantiate_args(fargs))
            # print "building rule for", svar
            node = LazyPNode(rule, dict(mapping), svar, rules)
            node.state = stat
            nodes.append(node)
        return nodes

    def to_actions(self, domain, parent_conds=None, parent_p=1.0, filter_func=None):
        return []

    def __str__(self):
        return "LazyPNode(%s): %s" % (self.rule.name, str(self.svar))
