import time
from itertools import chain, izip
from collections import defaultdict


import pddl
from pddl import utils, builtin, state, effects, dtpddl, mapl, visitors
from sas import *

def is_grounded(arg):
    if not isinstance(arg, pddl.types.Parameter):
        return True
    if not arg.is_instantiated() or isinstance(arg.get_instance(), pddl.types.Parameter):
        return False
    return True

def get_instance(a):
    if isinstance(a, pddl.types.Parameter) and a.is_instantiated():
        return a.get_instance()
    return a

def instantiate_args(args):
    result = []
    for arg in args:
        if arg.__class__ in (pddl.VariableTerm, pddl.types.Parameter) and arg.is_instantiated():
            result.append(arg.get_instance())
        elif isinstance(arg, (pddl.ConstantTerm, pddl.VariableTerm)):
            result.append(arg.object)
        elif isinstance(arg, pddl.types.TypedObject):
            result.append(arg)
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    return result

def match_facts(f1, f2, mapping, match_fn=utils.match_arguments):
    if isinstance(f1, LiftedFact):
        return f1.match(f2, mapping, match_fn)
    elif f1 == f2:
        return mapping.copy()
    return None
        
def match_svars(s1, s2, mapping, match_fn=utils.match_arguments):
    if isinstance(s1, LiftedStateVariable):
        return s1.match(s2, mapping, match_fn)
    elif s1 == s2:
        return mapping.copy()
    return None

def match_factlists(f1, f2, mapping, match_fn=utils.match_arguments):
    def match(list1, list2, mapping):
        if len(list1) != len(list2):
            return None
        for i, f1 in enumerate(list1):
            for j, f2 in enumerate(list2):
                m2 = match_facts(f1, f2, mapping, match_fn)
                if m2 is not None:
                    m2 = match(list1[:i] + list1[i+1:], list2[:j] + list2[j+1:], m2)
                    if m2 is not None:
                        return m2
            return None
        return mapping
    return match(f1, f2, mapping)

check_time = 0.0
    
class QueryNode(object):
    def __init__(self, wrapper, args , graph):
        assert all(isinstance(a, pddl.TypedObject) for a in args)
        self.graph = graph
        self.action = wrapper
        self.args = args
        self.mapping = dict(zip(self.action.args, self.args))
        self.instantiate()
        self.effects = [e.copy_instance() for e in self.action.effects]
        self.conditions = [e.copy_instance() for e in self.action.conditions]
        self.constraints = []
        self.children = []
        self.parents = []
        # print self
        # print map(type, args)
        self.uninstantiate()

    def instantiate(self):
        self.action.action.instantiate(self.mapping, self.graph)

    def uninstantiate(self):
        self.action.action.uninstantiate()

    def cyclic(self):
        def check_cycle(n):
            if n.action == self.action and n != self:
                return True
            return any(check_cycle(p) for p in n.parents)
        return check_cycle(self)

    def __str__(self):
        return "(%s %s)" % (self.action.action.name, " ".join(str(a) for a in self.args))

    def __hash__(self):
        return hash((self.action,) +  tuple(self.args))

    def __eq__(self, other):
        return hash(self) == hash(other) and self.action == other.action and \
            all(a == a2 for a,a2 in zip(self.args, other.args))

class DecisionNode(object):
    def __init__(self, svar, children, leafs=None):
        self.svar = svar
        self.children = children
        self.leafs = leafs
    
class QueryGraph(pddl.scope.Scope):
    def __init__(self, actions, domain):
        pddl.scope.Scope.__init__(self, [], domain)
        self.funcdict = defaultdict(list)
        self.paramcounts = defaultdict(lambda: 0)
        self.actiondict = defaultdict(list)
        self.nodedict = defaultdict(list)
        self.dtrees = {}
        self.all_actions = []
        
        for a in actions:
            for w in ActionWrapper.create(a):
                self.actiondict[a].append(w)
                self.prepare_dict(w)
                self.all_actions.append(w)
            
        for a in actions:
            for w in self.actiondict[a]:
                self.explore(w)
            
    def is_static(self, svar_or_fact):
        if isinstance(svar_or_fact, state.StateVariable):
            svar = svar_or_fact
        elif isinstance(svar_or_fact, state.Fact):
            svar = svar_or_fact.svar
        else:
            assert False
            
        try:
            return (svar.function, svar.modality) not in self.fluent_functions
        except:
            self.fluent_functions = set()
            for a in self.all_actions:
                for eff in a.effects:
                    self.fluent_functions.add((eff.svar.function, eff.svar.modality))
        # print map(str, self.fluent_functions)
        return (svar.function, svar.modality) not in self.fluent_functions

    def new_param(self, param):
        if isinstance(param.type, pddl.types.CompositeType):
            basename = "-".join(str(t) for t in param.type.types)
        else:
            basename = str(param.type)
            
        name = "?%s%d" % (basename, self.paramcounts[param.type])
        new_param = pddl.types.Parameter(name, param.type)
        self.add(new_param)
        self.paramcounts[param.type] += 1
        return new_param

    def get_relevant_constraints(self, fact, constraints):
        rel_args = set(fact.svar.all_args()) | set([fact.value])
        # print "rel:", map(str, rel_args)
        old_c = set()
        new_c = set(constraints)
        while old_c != new_c:
            old_c = new_c
            new_c = set()
            # print map(str, rel_args)
            for c in old_c:
                if not self.is_static(c):
                    # print "static:", function
                    continue
                # print c, map(str, args)
                if all(a in rel_args for a in c.svar.all_args()):
                    # rel_args |= set(chain(args, modal_args))
                    if isinstance(c.value, pddl.Parameter):
                        rel_args.add(c.value)
                    # if isinstance(value, pddl.ConstantTerm) or value.object in rel_args:
                    yield c
                else:
                    new_c.add(c)
                    # print "no rel:", c

    def get_all_isomorphic_nodes(self, node):
        def match_args(a1, a2, mapping):
            # print a1, a2, mapping.get(a1, None), hash(a1), id(a1)
            if not isinstance(a1, pddl.Parameter) or not isinstance(a2, pddl.Parameter):
                #constants must match exactly. No mapping from varaibles to constants allowed
                return a1 == a2
            return mapping.get(a1, a2) == a2
                        
        # print "node:",node
        # for c in node.constraints:
        #     print c
        # print 
            
        candidates = self.nodedict[node.action]
        for cn in candidates:
            if cn == node:
                continue
            if any(a1.get_type() != a2.get_type() for a1, a2 in zip(node.args, cn.args)):
                continue
            # print "-----match:",cn
            # for c in cn.constraints:
            #     print c
            # print 

            # mapping = dict(zip(node.args, cn.args))
            mapping = {}
            mapping = match_factlists(node.effects, cn.effects, mapping, match_fn=match_args)
            if mapping is not None:
                mapping = match_factlists(node.constraints, cn.constraints, mapping, match_fn=match_args)
            if mapping is not None:
                # print ["%s => %s" % (str(k),str(v)) for k,v in mapping.iteritems()]
                yield cn
                
    def get_isomorphic_node(self, node):
        try:
            return iter(self.get_all_isomorphic_nodes(node)).next()
        except StopIteration:
            return None

    def transitive_matching(self, conditions, constraints):
        conditions = filter(lambda c: self.is_static(c), conditions)
        constraints = filter(lambda c: self.is_static(c), constraints)
        for c in conditions:
            print "          |", c
        for c in constraints:
            print "          :", c
        
        def possibly_transitive(c):
            return len(c.svar.args) == 2 and c.svar.function.type == pddl.t_boolean and \
                c.svar.function.args[0].type == c.svar.function.args[1].type
        def generate_matches():
            for c in constraints:
                for c2 in conditions:
                    if c == c2:
                        continue
                    if possibly_transitive(c) and c.svar.function == c2.svar.function:
                        arg11 = c.svar.args[0]
                        arg12 = c.svar.args[1]
                        arg21 = c2.svar.args[0]
                        arg22 = c2.svar.args[1]
                        if arg12 == arg21:
                            yield arg12, arg22, c2
                        if  arg22 == arg11:
                            yield arg11, arg21, c2

        transitive_sets = defaultdict(set)
        for a1, a2, c in generate_matches():
            transitive_sets[(a1, a2)].add(c)

        if not transitive_sets:
            return list(set(c for c in chain(constraints, conditions) if self.is_static(c)))

        mapping = {}
        ignored = set()
        for (a1, a2), old_c in transitive_sets.iteritems():
            print "transitive mapping %s to %s. Facts: %s" % (a1, a2, " ".join(map(str, old_c)))
            if a2 in mapping:
                print "incompatible with %s=%s" % (a1, mapping[a2])
                continue
            
            m2 = mapping.copy()
            m2[a2] = a1
            for c in conditions:
                if c in old_c:
                    continue
                for c2 in constraints:
                    if match_svars(c2.svar, c.svar, m2) and not match_facts(c2, c, m2):
                        print "tr. mismatch:", c, c2
                        m2 = None
                        break
                if not m2:
                    break
            if m2 is not None:
                mapping = m2
                ignored |= old_c

        conditions = [c for c in conditions if c not in ignored]
        result = set(c for c in constraints if self.is_static(c))
        print ["%s => %s" % (k,v) for k,v in mapping.iteritems()]
        self.instantiate(mapping)
        new_conditions = []
        for c in conditions:
            if isinstance(c, LiftedFact):
                c_new = c.copy_instance()
                print c, c_new
            else:
                c_new = c
            new_conditions.append(c_new)
            result.add(c_new)
        self.uninstantiate()

        for c in result:
            print "          #", c
            
        return self.transitive_matching(new_conditions, constraints)

    def explore(self, action):
        args = [self.new_param(p) for p in action.args]
        node = QueryNode(action, args, self)
        node.constraints = [c for c in node.conditions if self.is_static(c)]
        # print node
        n2 = self.get_isomorphic_node(node)
        if n2:
            # print "already done:", n2
            return
        open_list = [node]
        closed = set()
        for n in open_list:
            if n in closed:
                continue
            closed.add(n)
            self.nodedict[n.action].append(n)
            # print "pop:", n
            # for e in n.effects:
            #     print "    ", e
            #     for c in self.get_relevant_constraints(e, chain(n.conditions, n.constraints)):
            #         print "          ", c
                        
            # constraints = self.transitive_matching(n.conditions, n.constraints)
            # for c in constraints:
            #     print "          |", c
            for ac, mapping in self.action_from_facts(n.effects, n.constraints):
                for a in ac.args:
                    if a not in mapping:
                        mapping[a] = self.new_param(a)
                    elif mapping[a] in n.mapping:
                        mapping[a] = n.mapping[mapping[a]]
                succ = QueryNode(ac, [mapping[a] for a in ac.args], self)
                # constraints = self.transitive_matching(succ.conditions, n.constraints)
                succ.constraints = list(c for c in chain(n.conditions, n.constraints) if self.is_static(c))
                # succ.constraints = constraints
                # iso = self.get_isomorphic_node(succ)
                # if iso:
                #     print "found isomorphic node"
                #     n.children.append(iso)
                #     iso.parents.append(n)
                # else:
                succ.parents.append(n)
                if not succ.cyclic():
                    n.children.append(succ)
                    open_list.append(succ)
                    # print " *", succ

    def collect_reachability_conds(self, function):
        pass
        
    def query_reachability(self, action, args, fact, problem, state_fn=None):
        t0 = time.time()
        nodes = []
        for a in self.actiondict[action]:
            nodes += self.nodedict[a]
            
        def search_node(n, f, mapping):
            # print "search:", n
            result = []
            for e in n.effects:
                m2 = e.match(f, mapping)
                if m2 is not None:
                    result.append((n, e, m2))
            for succ in n.children:
                result += search_node(succ, f, mapping)
            return result

        def constraints_match(constraints1, constraints2, eff1, eff2, canonical_mapping):
            #return True if constraints1 contains the same or fewer constraints1 than constraints2

            def canonical_matching(a1, a2, mapping):
                if not isinstance(a1, pddl.Parameter):
                    return a1 == a2
                if not a2.is_instance_of(a1.type):
                    return False
                if isinstance(a2, pddl.Parameter) and canonical_mapping.get(a1, None) != canonical_mapping.get(a2,None):
                    return False
                return mapping.get(a1, a2) == a2

            mapping = eff2.match(eff1, {}, canonical_matching)

            if mapping is None:
                return False
                
            for c2 in constraints2.itervalues():
                #need to find one constraint in c1 for each one in c2
                matched = False
                for c1 in constraints1.itervalues():
                    m2 = c2.match(c1, mapping, canonical_matching)
                    if m2 is not None:
                        # print "match:", c1, c2
                        matched = True
                        mapping = m2
                        break
                if not matched:
                    return False
            return True

        def dict_key(action, fact):
            return (action, fact.svar.function) + tuple(a.type for a in fact.all_args())

        if dict_key(action, fact) not in self.dtrees:
            csets = []

            svarsets = defaultdict(list)
            canonical_mapping = {}

            for n in nodes:
                # print n
                mapping = {}
                action_mapping = dict((utils.to_object(a), ("action", i)) for i, a in enumerate(n.args))
                for n2, eff, mapping in search_node(n, fact, mapping):
                    # print "  ",n2
                    constraints = dict((c.svar, c) for c in self.get_relevant_constraints(eff, chain(n2.conditions, n2.constraints)))
                    effect_mapping = dict((a, ("effect", i)) for i, a in enumerate(eff.all_args()))
                    new_canon_map = canonical_mapping.copy()
                    new_canon_map.update(effect_mapping)
                    new_canon_map.update(action_mapping)
                    
                    #Check if there are existing constraints that subsume this set
                    matched = False
                    new_csets = []
                    for i, (cset2, eff2) in enumerate(csets):
                        if constraints_match(constraints, cset2, eff, eff2, new_canon_map):
                            matched = True
                            new_csets.append((cset2, eff2))
                            # print "found existing match (superseding it)"
                        elif constraints_match(cset2, constraints, eff2, eff, new_canon_map):
                            pass
                            # print "found existing match"
                        else:
                            new_csets.append((cset2, eff2))
                    csets = new_csets
                    if not matched:
                        csets.append((constraints, eff))
                        canonical_mapping = new_canon_map

                        for c in constraints.itervalues():
                            svarsets[c.svar].append((constraints, eff))

            decision_vars = sorted(svarsets.keys(), key=lambda s: -len(svarsets[s]))
            # print map(str, decision_vars)
            # print ["%s = %s %d" % (a.name, tag, i) for a, (tag, i) in canonical_mapping.iteritems()]
            # print fact, "in", n

            # for constraints, eff in csets:
            #     print
            #     print map(str, constraints.itervalues())
            #     print eff

            def build_dtree(dvars, csets):
                if not csets:
                    return []
                if not dvars:
                    return [DecisionNode(None, None, csets)]

                svar = dvars[0]
                dvars = dvars[1:]
                child_dict = defaultdict(list)
                all_vals = []
                for constraints, eff in csets:
                    if svar not in constraints:
                        all_vals.append((constraints, eff))
                    else:
                        val = constraints[svar].value
                        if val.__class__ == pddl.TypedObject or val in canonical_mapping:
                            child_dict[val].append((constraints, eff))
                            # print "grounded:", eff, svar, val
                        else:
                            all_vals.append((constraints, eff))
                            # print "not grounded:", eff, svar, val
                all_trees = build_dtree(dvars, all_vals)
                
                node_children = [(val, build_dtree(dvars, childsets)) for val, childsets in child_dict.iteritems()]
                if not node_children or not any(nodes for _, nodes in node_children):
                    return all_trees
                # print "dvar:", svar
                return all_trees + [DecisionNode(svar, node_children, None)]

            def print_dtree(node, level=0):
                indent = "    " * level
                if node.leafs is not None:
                    print indent, "[%s]" % (", ".join(str(eff) for c,eff in node.leafs))
                else:
                    print indent, "%s" % str(node.svar)
                    for val, succ in node.children:
                        for n in succ:
                            print indent, "  %s" % str(val)
                            print_dtree(n, level+1)

            droots = build_dtree(decision_vars, csets)
            # print "--"
            # for node in droots:
            #     print_dtree(node)

            canonical_mapping = dict((k,v) for k,v in canonical_mapping.iteritems() if isinstance(k, pddl.Parameter))

            self.dtrees[dict_key(action, fact)] = (droots, canonical_mapping)
            
        droots, canonical_mapping = self.dtrees[dict_key(action, fact)]

        if not droots:
            # print "no trees"
            return False

        fact_args = list(fact.all_args())
        def get_mapping(tag, index):
            if tag == "action":
                return args[index]
            elif tag == "effect":
                return fact_args[index]

        mapping = dict((a, get_mapping(*v)) for a,v in canonical_mapping.iteritems() if get_mapping(*v).is_instance_of(a.type) )
        self.instantiate(mapping, problem)

        # print "\n", fact, "    ", action.name
        # print ["%s = %s" % (a, v) for a, v in mapping.iteritems()]
        # print ["%s = %s" % (repr(a), v) for a, v in mapping.iteritems()]
        
        def query_tree(node, f):
            if node.leafs:
                for _, eff in node.leafs:
                    if all(get_instance(a) == a2 for a, a2 in zip(eff.all_args(), f.all_args())):
                        # print "matching effect:", eff
                        return True
                # print "found no matching effect"
                return False
            
            val = state_fn(node.svar)
            for v, nodes in node.children:
                if get_instance(v) == val:
                    # print "%s = %s" % (str(node.svar), str(val))
                    return any(query_tree(n, f) for n in nodes)
            # print "no match for %s = %s" % (str(node.svar), str(val))
            # print "values are:", [str(get_instance(a)) for a,v in node.children]
            # print "values are:", map(repr, node.children.iterkeys()), repr(val)
            
            return False

        found = False
        for r in droots:
            if query_tree(r, fact):
                found = True
                # print "=======found==========="
                break

        self.uninstantiate()
        global check_time
        check_time += time.time() - t0
        return found

    def get_relevant_facts(self, action, args):
        pass
        
    def prepare_dict(self, action):
        for fact in action.conditions:
            function_arg = None
            # if any(isinstance(a, pddl.predicates.FunctionVariableTerm) for a in fact.svar.args):
            #     modality = lit.predicate
            #     function = None
            #     args = None
            #     neg = None
            #     modal_args = []
            #     for a in lit.args:
            #         if isinstance(a, pddl.predicates.FunctionVariableTerm):
            #             function_arg = a.object
            #         else:
            #             modal_args.append(a.object)
            #     value = pddl.TRUE if not lit.negated else pddl.FALSE
            # else:
            if True:
                # function, args, modality, modal_args, value, neg = utils.get_literal_elements(lit)
                if fact.svar.function.type == pddl.t_number:
                    continue
                function = fact.svar.function
                args = list(fact.svar.args)
                modality = fact.svar.modality
                modal_args = list(fact.svar.modal_args) 
                value = fact.value
                neg = fact.negated()

            if fact.value.type == pddl.t_number:
                continue
            # print modality, function, " => ", action

            self.funcdict[(modality, function)].append((args, modal_args, value, neg, function_arg, action))

    def action_from_literal(self, lit):
        function, args, modality, lit_modal_args, value, neg = utils.get_literal_elements(lit)
        # print lit, value
        #(modality, None) contains modal actions
        entries = chain(self.funcdict[(modality, function)], self.funcdict[(modality, None)])
        for svar_args, modal_args, val_arg, negated, func_arg, action in entries:
            # print "successors:", function, action
            mapping = {}
            #modal action:
            if func_arg is not None:
                assert modality is not None, lit
                mapping[func_arg] = pddl.Term(function, args)
                args_map = chain(zip(modal_args, lit_modal_args), [(val_arg, value)])
            else:
                args_map = chain(zip(svar_args, args), zip(modal_args, lit_modal_args), [(val_arg, value)])
                
            for arg, val in args_map:
                if isinstance(arg, pddl.Parameter) and val.is_instance_of(arg.type):
                    mapping[arg] = val
                # elif isinstance(val.object, pddl.Parameter) and arg.is_instance_of(arg.type):
                #     mapping[arg] = val
                elif arg != val.object:
                    mapping = None
                    # print "  mismatch: %s != %s" % (str(arg), str(val))
                    break
            if mapping:
                # print "gen:", fact, action.name, map(str, mapping.values())
                yield action, mapping


    def action_from_facts(self, facts, constraints):
        entries_by_action = defaultdict(list)
        constraints_by_action = defaultdict(list)
        for f in facts:
            #(modality, None) contains modal actions
            entries = chain(self.funcdict[(f.svar.modality, f.svar.function)], self.funcdict[(f.svar.modality, None)])
            for svar_args, modal_args, val_arg, negated, func_arg, action in entries:
                entries_by_action[action].append((f, svar_args, modal_args, val_arg, negated, func_arg))

        for f in constraints:
            entries = chain(self.funcdict[(f.svar.modality, f.svar.function)], self.funcdict[(f.svar.modality, None)])
            for svar_args, modal_args, val_arg, negated, func_arg, action in entries:
                constraints_by_action[action].append((f, svar_args, modal_args, val_arg, negated, func_arg))
                
        for action, entries in entries_by_action.iteritems():
            mapping = {}
            # print "succ:", action
            for f, svar_args, modal_args, val_arg, negated, func_arg in entries:
                # print "."
                # function, args, modality, lit_modal_args, value, neg = utils.get_literal_elements(lit)
                # print "successors:", function, action
                #modal action:
                if func_arg is not None:
                    assert modality is not None, lit
                    mapping[func_arg] = pddl.Term(f.svar.function, args)
                    args_map = chain(zip(modal_args, f.svar._modal_args), [(val_arg, f.value)])
                else:
                    args_map = chain(zip(svar_args, f.svar.args), zip(modal_args, f.svar.modal_args), [(val_arg, f.value)])

                for arg, val in args_map:
                    if not utils.match_arguments(arg, val, mapping):
                        # print "  mismatch in %s: %s => %s" % (str(f), str(arg), str(val))
                        mapping = None
                        break
                    if isinstance(arg, pddl.Parameter):
                        mapping[arg] = val
                    
                if mapping is None:
                    break

            for f, svar_args, modal_args, val_arg, negated, func_arg in constraints_by_action[action]:
                if mapping is None:
                    break
                
                args_map = chain(zip(svar_args, f.svar.args), zip(modal_args, f.svar.modal_args))
                if all(mapping.get(a, None) == a2 for a, a2 in args_map):
                    if not utils.match_arguments(val_arg, f.value, mapping):
                        # print "  c. mismatch in %s: %s => %s" % (str(f), str(arg), str(val))
                        mapping = None
                        break
                    if isinstance(val_arg, pddl.Parameter):
                        mapping[val_arg] = f.value
                
            if mapping:
                # print "gen:", fact, action.name, map(str, mapping.values())
                yield action, mapping
                
    def is_relevant(self, action, args, fact, state=None):
        pass
