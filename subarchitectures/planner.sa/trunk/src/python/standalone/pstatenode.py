from itertools import chain

import pddl
from pddl import state, effects, dtpddl, mapl

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
    def from_effect(peff, rules=[], stat=None):
        def get_children(eff):
            if isinstance(eff, pddl.ConjunctiveEffect):
                return eff.parts
            else:
                return [eff]

        def get_firing_rules(facts):
            for r in rules:
                deps = r.deps()
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in deps:
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
                deps = r.deps()
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in deps:
                        rule_facts[svar] = v
                if rule_facts:
                    yield (r, rule_facts)
            
        obj = pddl.TypedObject("node%d" % PNode.nodeid, dtpddl.t_node)
        PNode.nodeid += 1
        svar = state.StateVariable(dtpddl.selected, [obj])
            
        children = {}
        p_total = 0
        fsvar = fact.svar
        for i, (val, p) in enumerate(fact.value.iteritems()):
            facts = {fsvar : val}
            nodes = []
            for rule, fixed in get_firing_rules(facts):
                nodes += LazyPNode.from_rule(rule, rules, fixed, stat)
            val = pddl.TypedObject("choice%d" % i, dtpddl.t_node_choice)
                
            children[val] = (p, nodes, facts)
            p_total += p

        assert p_total <= 1.0001, "p(%s) = %.4f (%s)" % (str(svar), p_total, peff.pddl_str())
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
        if cval:
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
                
        rules = pddl.translators.Translator.get_annotations(problem.domain).get('dt_rules', [])
        nodes = []
        for e in peffs:
            nodes.append(PNode.from_effect(e, rules, s))
            
        return nodes

    @staticmethod
    def from_state(stat):
        rules = pddl.translators.Translator.get_annotations(stat.problem.domain).get('dt_rules', [])
        nodes = []
        for fact in stat.iterdists():
            if fact.value.value is None:
                nodes.append(PNode.from_fact(fact, rules, stat))
            
        return nodes
    
    def prepare_actions(self):
        pass
        # self.visited_by = set()
        # for val, (p, nodes, facts) in self.children.iteritems():
        #     for n in nodes:
        #         n.prepare_actions()

    def to_actions(self, domain, parent_conds=None, parent_p=1.0):
        actions = []
        # if parent_conds:
        #     if frozenset(parent_conds) in self.visited_by:
        #         return []
        #     self.visited_by.add(frozenset(parent_conds))

        for o in self.svar.args:
            domain.add_constant(o)
            
        for val, (p, nodes, facts) in self.children.iteritems():
            if p * parent_p <= 0.001:
                # print "pruned due to likelihood"
                continue
            name = "commit-%s-%s-%s" % (self.svar.function.name, "-".join(a.name for a in self.svar.args), val.name)
            PNode.actionid += 1
            domain.add_constant(val)
                
            agent = pddl.Parameter("?a", mapl.t_agent)
            a = mapl.MAPLAction(name, [agent], [], [], pddl.Conjunction([]), None, pddl.ConjunctiveEffect([]), [], domain)
            actions.append(a)
            b = pddl.Builder(a)
            a.precondition.parts.append(b.cond("not", ("committed", self.svar.as_term())))
            # a.precondition.parts.append(b.cond("not", ("started",)))
            if parent_conds:
                a.precondition.parts += [c.copy(a) for c in parent_conds]

            a.effect.parts.append(b.effect("assign", self.svar.as_term(), val))
            for svar, value in facts.iteritems():
                for c in chain(svar.args, [value]):
                    domain.add_constant(c)
                cvar = svar.as_modality(mapl.commit, [value])
                a.effect.parts.append(cvar.as_literal(_class=effects.SimpleEffect))
                
            a.effect.parts.append(b.effect("assign", ("probability",), p))
            a.set_total_cost(0)

            new_parent_cond = b.cond("=", self.svar.as_term(), val)

            for n in nodes:
                actions += n.to_actions(domain, [new_parent_cond], p * parent_p)

        return actions

    def to_init(self, selected_facts=None):
        def make_unknown(val):
            return pddl.TypedObject("other-%s" % val.type.name, val.type)
        
        selected_svars = set(f.svar for f in selected_facts) if selected_facts is not None else None
        effs = []
        for val, (p, nodes, facts) in self.children.iteritems():
            ceff = pddl.ConjunctiveEffect([])
            for svar, val in chain(facts.iteritems(), [(self.svar, val)]):
                #if svar.function == selected:
                #    continue
                f = state.Fact(svar, val)
                if selected_facts is not None and f not in selected_facts:
                    # if f.svar in selected_svars and f.svar.function != selected:
                    #     f = state.Fact(svar, make_unknown(val))
                    # else:
                    continue
                eff = f.as_literal(_class=pddl.SimpleEffect)
                ceff.parts.append(eff)
            for n in nodes:
                eff = n.to_init(selected_facts)
                if eff:
                    ceff.parts.append(eff)
            if ceff.parts:
                effs.append((pddl.Term(p), ceff))
        if effs:
            return effects.ProbabilisticEffect(effs)
        return None

    def __str__(self):
        return "PNode: %s" % str(self.svar)

        
class LazyPNode(PNode):
    def __init__(self, rule, mapping, branch, all_rules):
        self.rule = rule
        self.all_rules = all_rules
        self.mapping = mapping
        self.parent = None
        self._children = None
        self.svar = branch

    def simplify(self):
        return [], {}, False
        
    def get_children(self):
        if self._children is not None:
            return self._children

        self._children = {}
        # print ["%s = %s" % (str(k),str(v)) for k,v in self.mapping.iteritems() ]

        def get_firing_rules(facts):
            for r in self.all_rules:
                deps = r.deps()
                rule_facts = {}
                for svar,v in facts.iteritems():
                    if svar.function in deps:
                        rule_facts[svar] = v
                if rule_facts:
                    yield (r, rule_facts)
        
        def get_objects(arg):
            if arg in self.mapping:
                return [self.mapping[arg]]
            return list(self.state.problem.get_all_objects(arg.type))
        
        args = self.rule.args + self.rule.add_args
        # print "creating subtree for %s" % str(self.svar)
        # print ["%s = %s" % (str(k),str(v)) for k,v in self.mapping.iteritems() ]
        for mapping in self.rule.smart_instantiate(self.rule.get_inst_func(self.state), args, [get_objects(a) for a in args], self.state.problem):
            # log.debug("creating subtree for %s", str(self.svar))
            
            # print "  ", ["%s = %s" % (str(k),str(v)) for k,v in mapping.iteritems() ]
            for p, value in self.rule.values:
                p = self.state.evaluate_term(p)
                if p == pddl.UNKNOWN or p.value < 0.01:
                    continue
                val = self.state.evaluate_term(value)
                facts = {self.svar : val}
                # print "  generating child for %s = %s (p=%.2f)" % (str(self.svar), str(val), p.value)

                nodes = []
                for rule, fixed in get_firing_rules(facts):
                    nodes += LazyPNode.from_rule(rule, self.all_rules, fixed, self.state)


                self._children[val] = (p.value, nodes, facts)
        return self._children
            
    children = property(get_children)
        

    @staticmethod
    def from_rule(rule, rules, fixed_facts, stat):
        stat = stat.copy()
        for svar, val in fixed_facts.iteritems():
            stat[svar] = val
            
        pre_mapping = rule.match_args(fixed_facts.iteritems())
        if pre_mapping is None:
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
            if arg in pre_mapping:
                return [pre_mapping[arg]]
            return list(stat.problem.get_all_objects(arg.type))

        nodes = []
        args = rule.args + list(a for a in pre_mapping.iterkeys() if a not in rule.args)
        for mapping in rule.smart_instantiate(rule.get_inst_func(stat), args, [get_objects(a) for a in args], stat.problem):
            svar = state.StateVariable(rule.function, state.instantiate_args(rule.args))
            # print "building rule for", svar
            node = LazyPNode(rule, dict(mapping), svar, rules)
            node.state = stat
            nodes.append(node)
        return nodes

    def to_actions(self, domain, parent_conds=None, parent_p=1.0):
        return []

    def __str__(self):
        return "LazyPNode(%s): %s" % (self.rule.name, str(self.svar))
