from collections import defaultdict
import itertools
import config

import pddl
from pddl import state, visitors, mapl
import plans
import assertions

log = config.logger("assertions")

ATOM_DISABLED = 1
ATOM_ENABLED = 2
ATOM_ALL = ATOM_ENABLED | ATOM_DISABLED

class MacroOp(pddl.scope.Scope):
    def __init__(self, name, agent_param, args, pre, eff, domain):
        pddl.scope.Scope.__init__(self, args + [agent_param], domain)
        self.name = name
        self.agent = agent_param
        self.args = args
        self.pre = pre
        self.eff = eff
        self.domain = domain

        self.costs = 1
        self.usecount = 0
        self.subsumption_count = 1
        self.alpha_boost = 1

        self.atom_state = {}
        self.Q = {}
        self.frequencies = {}
        self.init_structures()

    def q_total(self):
        total = 0.0
        for a, q in self.Q.iteritems():
            count = self.frequencies[a]
            total += q*count
        return (total/self.usecount) / (len(self.pre)+len(self.eff))

    def q_expected(self):
        """Return the expected value of this macros if this macro is used with the current
        configuration of preconditions and effects.
        In contrast to q_total, this method will set the probabilities of enabled preconditions to 1.0 and
        those of diables effects to 0.0"""
        total = 0.0
        for a in self.pre:
            if self.atom_state[a] == ATOM_ENABLED:
                total += self.usecount * self.Q[a]
            else:
                for a2 in (a, a.negate()):
                    total += self.frequencies[a2] * self.Q[a2]
                    
        for a in self.eff:
            if self.atom_state[a] == ATOM_DISABLED:
                total += self.usecount * self.Q[a.negate()]
            else:
                for a2 in (a, a.negate()):
                    total += self.frequencies[a2] * self.Q[a2]
                    
        return (total/self.usecount) / (len(self.pre)+len(self.eff))
    
    @staticmethod
    def parse(it, scope):
        it.get(":macro")
        action = mapl.MAPLAction.parse(iter(it.get(list, "action definition")), scope)
        macro = MacroOp.from_action(action, scope)

        costs = -1
        freq = -1

        ops = [pddl.assign]
        if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
            ops += pddl.numeric_ops

        scope.predicates.add(ops)

        for elem in it:
            j = iter(elem)
            first = j.get()
            
            if first.is_terminal():
                val = j.get("terminal", "value").token
                try:
                    value = float(val.string)
                except:
                    raise pddl.parser.UnexpectedTokenError(val, "number")

                if first.token.string == ":usecount":
                    macro.usecount = float(value)
                elif first.token.string == ":subsumes":
                    macro.subsumption_count = int(value)
                elif first.token.string == ":costs":
                    macro.costs = float(value)
            else:
                atom = pddl.Literal.parse(iter(first), macro)
                if atom not in macro.Q:
                    raise pddl.parser.ParseError(first.token, "Unknown atom: %s" % str(atom))
                state = int(j.get().token.string)
                count = float(j.get().token.string)
                val = float(j.get().token.string)
                if atom in macro.atom_state:
                    macro.atom_state[atom] = state
                macro.frequencies[atom] = count
                macro.Q[atom] = val

        scope.predicates.remove(ops)
        return macro

    def init_structures(self):
        self.atom_state = dict([(a, ATOM_DISABLED) for a in self.pre] + [(a, ATOM_ENABLED) for a in self.eff])

        self.Q = dict((a, 0) for a in itertools.chain(self.pre, self.eff))
        self.Q.update(dict((a.negate(), 0) for a in itertools.chain(self.pre, self.eff)))
        
        self.frequencies = dict((a, 0) for a in itertools.chain(self.pre, self.eff))
        self.frequencies.update(dict((a.negate(), 0) for a in itertools.chain(self.pre, self.eff)))

    def normalize_preconditions(self):
        kval = {}
        indomain = {}
        candidates = set()
        for pre in self.pre:
            if pre.predicate == mapl.knowledge:
                term = pre.args[1]
                kval[term] = pre
                candidates.add(term)
            if pre.predicate == mapl.indomain:
                term = pre.args[0]
                indomain[term] = pre
                candidates.add(term)
                
        for term in candidates:
            if term in kval and term in indomain:
                self.pre.remove(kval[term])
                self.pre.remove(indomain[term])
                val = indomain[term].args[1]
                new = pddl.LiteralCondition(pddl.equals, [term, val], self)
                self.pre.append(new)

    @staticmethod
    def from_action(action, domain):
        @visitors.collect
        def fact_collector(elem, results):
            if isinstance(elem,  pddl.Literal):
                return elem
            if not isinstance(elem, (pddl.Conjunction, pddl.ConjunctiveEffect)):
                assert False, "Unsupported condition: %d" % cond.pddl_str()

        macro = MacroOp(action.name, action.agents[0], action.maplargs+action.vars, [], [], domain)
        for pre in action.precondition.visit(fact_collector):
            macro.pre.append(pre.copy(macro))
            
        for pre in visitors.visit(action.replan, fact_collector, []):
            macro.pre.append(pre.copy(macro))

        for e in action.effect.visit(fact_collector):
            macro.eff.append(e.copy(macro))
            
        macro.normalize_preconditions()
        macro.init_structures()
        return macro

                
    @staticmethod
    def from_cluster(cluster, plan, domain, name):
        def find_fact(node, svar, value, where='read'):
            @visitors.collect
            def visitor(elem, results):
                if isinstance(elem, pddl.Literal):
                    svar2, val2 = state.Fact.from_literal(elem)
                    if svar == svar2 and (value == val2 or value is None):
                        return elem
                elif not isinstance(elem, (pddl.Conjunction, pddl.ConjunctiveEffect)):
                    assert False, "unsupported condition or effect: %s" % str(elem)

            if where == 'read':
                if svar.modality == mapl.direct_knowledge:
                    svar = svar.as_modality(mapl.knowledge, svar.modal_args)
                elif svar.modality == mapl.i_indomain:
                    svar = svar.as_modality(mapl.indomain, svar.modal_args)
            elif where == 'written':
                if svar.modality == mapl.knowledge:
                    svar = svar.as_modality(mapl.direct_knowledge, svar.modal_args)

            result = []
            node.action.instantiate(node.args)
            if where == 'read':
                result += visitors.visit(node.action.precondition, visitor, [])
                result += visitors.visit(node.action.replan, visitor, [])
            elif where == 'written':
                result += node.action.effect.visit(visitor)
            node.action.uninstantiate()
            
            #try find a modal variant
            if not result and svar.modality == None and where == 'read':
                log.debug("couldn't find literal for %s=%s, trying in-domain modality", svar, value)
                result = find_fact(node, svar.as_modality(mapl.indomain, [value]), pddl.TRUE)
                if not result:
                    log.debug("couldn't find literal for %s=%s, trying knowledge modality", svar, value)
                    result = find_fact(node, svar.as_modality(mapl.knowledge, [node.args[0]]), pddl.TRUE)
                
            return result

        def get_mapping(a1, a2):
            def collect_args(term, results):
                if isinstance(term, (pddl.VariableTerm, pddl.ConstantTerm)):
                    return [term.object]
                if isinstance(term, pddl.FunctionTerm):
                    return sum(results, [])
                
            for arg, arg2 in zip(a1.collect_arguments(), a2.collect_arguments()):
                yield (arg, arg2)

        def copy_atom(atom, mapping, scope=None):
            def copyTerm(term, results):
                if isinstance(term, pddl.VariableTerm):
                    return pddl.Term(mapping[term.object])
                elif isinstance(term, pddl.ConstantTerm):
                    return pddl.Term(term.object)
                elif isinstance(term, pddl.FunctionTerm):
                    return pddl.Term(term.function, results)
            if isinstance(atom, pddl.LiteralCondition):
                return pddl.LiteralCondition(atom.predicate, [a.visit(copyTerm) for a in atom.args])
            elif isinstance(atom, pddl.SimpleEffect):
                return pddl.SimpleEffect(atom.predicate, [a.visit(copyTerm) for a in atom.args])
                
        log.debug("extracting operator from cluster %s", map(str,cluster))
                                
        agent_param = pddl.Parameter("?a", mapl.t_agent)
        mapping = {agent_param : agent_param}
        args = set([agent_param])

        frontier = {}
        read = {}
        written = {}
        
        for i, pnode in enumerate(plan.topological_sort()):
            if pnode not in cluster or isinstance(pnode, plans.DummyNode):
                continue
            log.debug("processing node %d: %s", i, str(pnode))
            action = pnode.action
            action.instantiate(pnode.args)

            local_map = {}
            for param in action.args:
                new = pddl.Parameter(param.name+"#"+str(i), param.type)
                assert new not in mapping
                args.add(new)
                mapping[new] = new
                local_map[param] = new

            args.remove(local_map[action.agents[0]])
            if local_map[action.agents[0]].type.is_subtype_of(agent_param.type):
                del mapping[agent_param]
                args.remove(agent_param)
                agent_param.type = local_map[action.agents[0]].type
                mapping[agent_param] = agent_param
                args.add(agent_param)
            mapping[local_map[action.agents[0]]] = agent_param

            pairs = []

            for svar, val in itertools.chain(pnode.preconds, pnode.replanconds):
                if val == pddl.UNKNOWN:
                    continue
                
                atom = copy_atom(find_fact(pnode, svar, val)[0], local_map)
                #print atom.pddl_str()
                if svar in frontier:
                    pairs.append((atom, frontier[svar]))
                if svar not in written:
                    read[svar] = atom
                frontier[svar] = atom

            for svar, val in pnode.effects:
                atom = copy_atom(find_fact(pnode, svar, val, where='written')[0], local_map)
                written[svar] = atom
                frontier[svar] = atom

            for a1, a2 in pairs:
                for arg, arg2 in get_mapping(a1, a2):
                    if arg not in domain:
                        arg = mapping[arg]
                    if arg2 not in domain:
                        arg2 = mapping[arg2]

                    if arg == arg2:
                        continue
                    log.debug("possible matches: %s, %s", arg, arg2)
                        
                    if arg in domain and arg2 not in domain:
                        old = arg2
                        new = arg
                    elif arg2 in domain and arg not in domain:
                        old = arg
                        new = arg2
                    elif arg2 not in domain and arg not in domain:
                        old = arg
                        new = arg2
                    else:
                        continue
                    
                    log.debug("merging %s with %s", old, new)
                    args.remove(old)
                    mapping[old] = new
                    for key, val in mapping.iteritems():
                        if val == old:
                            mapping[key] = new
                
            log.debug("parameter set is now: %s", " ".join(a.name for a in args))

        args.remove(agent_param)
        macro = MacroOp(name, agent_param, list(args), [], [], domain)
        
        for svar, atom in read.iteritems():
            new = copy_atom(atom, mapping, macro)
            log.debug("adding precondition: %s => %s", atom.pddl_str(), new.pddl_str())
            macro.pre.append(new)
            
        for svar, atom in written.iteritems():
            if svar in read and read[svar] == atom:
                continue
            new = copy_atom(atom, mapping, macro)
            log.debug("adding effect: %s => %s", atom.pddl_str(), new.pddl_str())
            macro.eff.append(new)

        #create sensible argument names
        names = set()
        for arg in macro.args:
            oldname = arg.name
            arg.name = arg.name.split("#")[0]
            if arg.name in names:
                basename = arg.name
                while basename[-1] in '0123456789':
                    basename = basename[:-1]
                i = 1
                while basename+str(i) in names:
                    i += 1
                arg.name = basename+str(i)
            del macro[oldname]
            
            macro.add(arg)
            names.add(arg.name)

        macro.normalize_preconditions()
        macro.init_structures()
        return macro


    def less_than(self, other, enabled_only=False):
        """ Return 'True' if other can be always applied instead of this macro."""
        def count_mapped_vars(mapping):
            return len([x for x in mapping.iterkeys() if x in other.args])

        def rel(macro, atoms):
            if enabled_only:
                return filter(lambda a: macro.atom_state[a] == ATOM_ENABLED, atoms)
            return atoms

        predicate_equalities = set([
            (pddl.equals, pddl.assign),
            (pddl.builtin.eq, pddl.builtin.num_assign)])

        def try_map_atoms(atom1, atom2, mapping):
            """ try to map the atoms 1 and 2 to each other (where 2 must be a
            weaker or equal precondition or stronger equal effect) """

            def map_term(term1, term2):
                if term1.__class__ != term2.__class__:
                    return False
                if term1.__class__ == pddl.ConstantTerm:
                    return term1.object == term2.object
                if term1.__class__ == pddl.VariableTerm:
                    if not term1.get_type().equal_or_subtype_of(term2.get_type()):
                        return False
                    if mapping.get(term1.object, term2.object) != term2.object:
                        log.debug("Conflict when trying to map %s to %s, but it's already mapped to %s", term1.pddl_str(), term2.pddl_str(), mapping[term1.object])
                        return False
                    if mapping.get(term2.object, term1.object) != term1.object:
                        log.debug("Conflict when trying to map %s to %s, but it's already mapped to %s", term2.pddl_str(), term1.pddl_str(), mapping[term2.object])
                        return False
                    log.debug("Mapping %s to %s", term1.pddl_str(), term2.pddl_str())
                    mapping[term1.object] = term2.object
                    mapping[term2.object] = term1.object
                    return True
                if term1.__class__ == pddl.FunctionTerm:
                    if term1.function != term2.function:
                        return False
                    log.debug("Trying to map %s to %s", term1.pddl_str(), term2.pddl_str())
                    return all(map_term(t1,t2) for t1,t2 in zip(term1.args, term2.args))
                assert False

            #simple case: normal predicates
            #if not atom1.predicate.builtin and not atom2.predicate.builtin:
            log.debug("predicates are: %s and %s", atom1.predicate, atom2.predicate)
            if atom1.predicate != atom2.predicate and (atom1.predicate, atom2.predicate) not in predicate_equalities:
                return False
                
            return all(map_term(t1,t2) for t1,t2 in zip(atom1.args, atom2.args))
        
        def backtrack(a1, a2, index, mapping):
            #if count_mapped_vars(mapping) == len(a2.args):
            #    return True

            a2_pre_count = len(rel(a2, a2.pre))

            if index >= a2_pre_count:
                log.debug("Looking at effects")
                #Atoms in eff(A2) and pre(A1) can be mapped on each other!
                a1_atoms = rel(a1, a1.pre) + rel(a1, a1.eff)
                a2_atoms = rel(a2, a2.eff)
                offset = a2_pre_count
            else:
                log.debug("Looking at preconditions")
                a1_atoms = rel(a1, a1.pre)
                a2_atoms = rel(a2, a2.pre)
                offset = 0

            atom2 = None
            next_index = -1
            for i in xrange(index-offset, len(a2_atoms)):
                next_index = i+offset
                log.debug("considering atom #%d", i)
                #if not all(arg in mapping for arg in a2_atoms[i].collect_arguments()):
                atom2 = a2_atoms[i]
                break

            if atom2 is None:
                #We are already done with the preconditions (of A2). Continue with the effects
                if index < a2_pre_count:
                    return backtrack(a1, a2, a2_pre_count, mapping)
                log.debug("No more atoms in a2 to map")
                return True

            for atom1 in a1_atoms:
                #print atom1, " <-> ", atom2
                #if atom1.predicate != atom2.predicate
                #    continue
                #if not any(arg.object in mapping for arg in atom1.args+(atom1.value,)):
                #    continue
                new_mapping = mapping.copy()
                log.debug("Try new assignment:")
                if try_map_atoms(atom1, atom2, new_mapping):
                    log.debug("success")
                    if backtrack(a1, a2, next_index+1, new_mapping):
                        mapping.update(new_mapping)
                        return True
                    log.debug("Backtracking...")
                else:
                    log.debug("failed")

            log.debug("No solution in this branch")
            return False

        if len(other.args) > len(self.args):
            log.debug("a1 has less parameters than a2.")
            return False

        mapping = {}
        if not backtrack(self, other, 0, mapping):
            log.debug("Mapping failed")
            return False

        #check if all effects of self are satisfied:
        for eff in rel(self, self.eff):
            if not any(try_map_atoms(eff, eff2, mapping) for eff2 in rel(other, other.eff)):
                log.debug("Effect %s is not in a2." % eff.pddl_str())
                return False
            
        return True

    def get_satisfied_conditions(self, pnode, thisstate):
        def uninstantiated(term):
            return any(arg.__class__ == pddl.Parameter and not arg.is_instantiated() for arg in term.visit(pddl.predicates.collect_args_visitor))
        def num_uninstantiated(atom):
            args = set()
            for arg in atom.collect_arguments():
                if arg.__class__ == pddl.Parameter and not arg.is_instantiated():
                    args.add(arg)
            return len(args)

        def get_possible_svars(function, args, value=None):
            for svar, val in thisstate.iteritems():
                if svar.function == function and (value is None or value == val):
                    mapping = {}
                    for arg1, arg2 in itertools.izip(args, svar.args):
                        if arg1.is_instantiated() and arg1.get_instance() != arg2:
                            mapping = None
                            break
                        elif not arg2.is_instance_of(arg1.get_type()):
                            mapping = None
                            break
                        elif not arg1.is_instantiated():
                            mapping[arg1.object] = arg2
                    if mapping:
                        yield svar, mapping

        def get_candiates(atom):
            known = []
            unknown = []
            for term in atom.args:
                if uninstantiated(term):
                    unknown.append(term)
                else:
                    if term.__class__ == pddl.FunctionTerm:
                        known.append(thisstate[state.StateVariable.from_term(term)])
                    elif term.__class__ == pddl.VariableTerm:
                        known.append(term.get_instance())
                    elif term.__class__ == pddl.ConstantTerm:
                        known.append(term.object)

            mapping = {}
            if atom.predicate in (pddl.equals, pddl.builtin.eq):
                if not known or not unknown:
                    return
                
                if unknown[0].__class__ == pddl.VariableTerm:
                    yield {unknown[0].object : known[0]}
                else:
                    for svar, mapping in get_possible_svars(unknown[0].function, unknown[0].args, known[0]):
                        yield mapping
                        
            elif not atom.predicate.builtin:
                for svar, mapping in get_possible_svars(atom.predicate, atom.args, pddl.TRUE ):
                    yield mapping
                

        mapping = dict((param.name, c) for (param, c) in zip(pnode.action.args, pnode.args))
        self.instantiate(mapping)
        
        partially_instantiated = [pre for pre in self.pre if num_uninstantiated(pre) > 0]
        log.debug("partially instantiated: %s", " ".join(p.pddl_str() for p in partially_instantiated))

        added = True
        while added:
            added = False
            for atom in partially_instantiated:
                log.debug("looking at: %s", atom.pddl_str())
                for poss_mapping in get_candiates(atom):
                    log.debug("trying mapping: %s", " ".join("%s => %s" % (k,v) for k,v in poss_mapping.iteritems()))
                    new_mapping = mapping.copy()
                    new_mapping.update(poss_mapping)
                    
                    self.uninstantiate()
                    self.instantiate(new_mapping)

                    consistent = True
                    for atom2 in partially_instantiated:
                        if num_uninstantiated(atom2) == 0:
                            if not thisstate.is_satisfied(atom2):
                                log.debug("inconsistent: %s is not satisfied", atom2.pddl_str())
                                consistent = False
                                break
                    if consistent:
                        log.debug("consistent")
                        mapping = new_mapping
                        added = True
                    else:
                        self.uninstantiate()
                        self.instantiate(mapping)
                        
            if added:
                partially_instantiated = [pre for pre in self.pre if num_uninstantiated(pre) > 0]
                
        log.debug("satisfied:")
        for p in self.pre:
            if num_uninstantiated(p) == 0:
                log.debug(p.pddl_str())
            
        return [p for p in self.pre if num_uninstantiated(p) == 0 and thisstate.is_satisfied(p)]

    def to_action(self, filter=ATOM_ENABLED):
        precondition = pddl.Conjunction([p for p in self.pre if self.atom_state[p] & filter])
        effect = pddl.ConjunctiveEffect([eff for eff in self.eff if self.atom_state[p] & filter])
        
        return mapl.MAPLAction(self.name, [self.agent], self.args, [], precondition, None, effect, self.domain).copy()

    def to_assertion(self):
        observable = assertions.get_observable_functions(self.domain.sensors)
        precondition = pddl.Conjunction([])
        replan = pddl.Conjunction([])
        
        for p in self.pre:
            if p.predicate == mapl.knowledge:
                replan.parts.append(p)
            elif p.predicate == pddl.equals and all(not isinstance(t, pddl.FunctionTerm) for t in p.args):
                if self.atom_state[p] == ATOM_ENABLED:
                    precondition.parts.append(p)
            elif p.predicate != pddl.equals or \
                    not assertions.is_observable(observable, p.args[0], p.args[1]):
                if self.atom_state[p] == ATOM_ENABLED:
                    precondition.parts.append(p)
            else:
                if self.atom_state[p] == ATOM_ENABLED:
                    id_cond = pddl.LiteralCondition(mapl.indomain, p.args[:], negated = p.negated)
                    precondition.parts.append(id_cond)
                k_cond = pddl.LiteralCondition(mapl.knowledge, [pddl.VariableTerm(self.agent), p.args[0]])
                replan.parts.append(k_cond)
                    
        effect = pddl.ConjunctiveEffect([eff for eff in self.eff if self.atom_state[eff] == ATOM_ENABLED])
        costs = max(1, self.costs)
        cost_eff = pddl.SimpleEffect(pddl.builtin.increase, [pddl.Term(pddl.builtin.total_cost, []), pddl.Term(costs-1)])
        #effect.parts.append(cost_eff)

        args = set()
        for atom in itertools.chain(precondition.parts, replan.parts, effect.parts):
            for arg in atom.collect_arguments():
                if arg.__class__ == pddl.Parameter and arg != self.agent:
                    args.add(arg)
        
        return mapl.MAPLAction(self.name, [self.agent], list(args), [], precondition, replan, effect, self.domain).copy()


class MacroWriter(mapl.MAPLWriter):
    
    def write_macro(self, macro):
        strings = self.write_action(macro.to_action(filter=ATOM_ALL))
        strings.append("(:usecount %f)" % macro.usecount)
        strings.append("(:subsumes %d)" % macro.subsumption_count)
        strings.append("(:costs %f)" % macro.costs)
        for atom in itertools.chain(macro.pre, macro.eff):
            strings.append("(%s %d %f %f)" % (self.write_literal(atom), macro.atom_state[atom], macro.frequencies[atom], macro.Q[atom]))
            natom = atom.negate()
            strings.append("(%s 0 %f %f)" % (self.write_literal(natom), macro.frequencies[natom], macro.Q[natom]))
            
        return self.section(":macro", strings)

    def write_macros(self, macros):
        strings = ["("]
        for m in macros:
            strings += self.write_macro(m)
            strings.append("")
        strings.append(")")
        return strings

def load_macros(filename, domain):
    p = pddl.parser.Parser.parse_file(filename)
    result = []
    for elem in iter(p.root):
        result.append(MacroOp.parse(iter(elem), domain))
    return result
