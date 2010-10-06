import os, time
import itertools

from collections import defaultdict
from standalone import task, config, pddl, plans
from standalone.pddl import state, dtpddl, mapl, translators, visitors, effects

import standalone.globals as global_vars
import partial_problem

log = config.logger("dt")


class DTProblem(object):
    def __init__(self, plan, domain, cast_state):
        self.plan = plan
        self.domain = domain
        self.state = cast_state
        self.subplan_actions = []
        self.select_actions = []
        self.selected_subproblem = -1
        
        self.goals = self.create_goals(plan)
        if not self.goals:
            return
        
        self.goal_actions = []
        self.relaxation_layers = self.compute_restrictions()
        self.dt_rules = pddl.translators.Translator.get_annotations(domain).get('dt_rules', [])
        
        self.dtdomain = self.create_dt_domain(domain)
        self.goal_actions += self.create_goal_actions(self.goals, self.dtdomain)
        self.dtdomain.actions += [a for a in self.goal_actions]
        self.dtdomain.name2action = None
        
        self.subproblems = self.compute_subproblems(self.state)
        self.problem = self.create_problem(self.state, self.dtdomain)
        self.dt_plan = []

        # dom_str, prob_str = DTPDDLOutput().write(self.problem)
        # print "\n".join(dom_str)
        # print "\n".join(prob_str)

    def write_dt_input(self, domain_fn, problem_fn):
        DTPDDLOutput().write(self.problem, domain_fn=domain_fn, problem_fn=problem_fn)

    def subplan_active(self, plan):
        if not self.subplan_actions:
            return False
        
        for pnode in plan.topological_sort():
            if pnode in self.subplan_actions:
                if pnode.status == plans.ActionStatusEnum.EXECUTED:
                    self.subplan_actions.remove(pnode)
                else:
                    return True
            if pnode.status != plans.ActionStatusEnum.EXECUTED:
                return False
        assert False, "DT-Subplan actions no longer in plan!"

    def create_goals(self, plan):
        observe_actions = translators.Translator.get_annotations(self.domain).get('observe_effects', [])
        if not observe_actions:
            return []

        def find_restrictions(pnode):
            others = []
            for pred in plan.predecessors_iter(pnode, 'depends'):
                restr = find_restrictions(pred)
                if pred.action.name.startswith("select-"):
                    return [pred] + restr
                if restr:
                    others = restr
            return others

        goal_svars = set()
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            if pnode.action.name in observe_actions:
                #print pnode.action.name
                #print map(str, pnode.effects)
                #TODO: only add an action if the observe effect supports a later action
                for svar, val in pnode.effects:
                    if svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                        goal_svars.add(svar.nonmodal())
                self.subplan_actions.append(pnode)
                self.select_actions = [pnode] + find_restrictions(pnode)
                break # only one action at a time for now
            
            if pnode.action.name not in observe_actions and self.subplan_actions:
                break
            
        return goal_svars
    

    def replanning_neccessary(self, new_state):
        if self.selected_subproblem == -1:
            return True
        new_objects = new_state.objects - self.state.objects
        problem = self.subproblems[self.selected_subproblem]
        for o in new_objects:
            if all(c.matches(o, self.state.prob_state) for c in problem.constraints):
                return True
        return False
    

    def compute_restrictions(self):

        def transform_constraints(new_fixed, new_constraints):
            c2 = []
            replace_dict = {}
            for obj, cset in new_constraints.iteritems():
                for c in cset:
                    if obj in replace_dict or obj in new_fixed:
                        continue
                    args = []
                    assert not c.svar.modal_args, "Not yet supported."
                    for a in c.svar.args:
                        a2 = pddl.TypedObject("any_%s" % (a.type.name), a.type)
                        replace_dict[a] = a2
                        args.append(a2)
                    val = replace_dict.get(c.value, c.value)
                    constr = partial_problem.FunctionConstraint(c.svar.function, args, [val])
                    if constr not in c2:
                        c2.append(constr)
            return c2
            
        layers = []
        var_relaxations = set()
        value_relaxations = set()
        fixed = set()
        constraints = defaultdict(set)
        for pnode in self.select_actions:
            log.debug("action: %s", str(pnode))
            log.debug("cond: %s", str(map(str,pnode.preconds)))
            supporter = {}
            for pred in self.plan.predecessors_iter(pnode, link_type='depends'):
                for e in self.plan[pred][pnode].itervalues():
                    for svar, val in pnode.preconds:
                        if e['svar'] == svar:
                            supporter[svar] = pred
                            break
                        
            new_fixed = fixed | set(pnode.full_args)
            new_var_relaxations = set()
            new_value_relaxations = set()
            new_constraints = defaultdict(set)
            for svar, val in pnode.preconds:
                if svar.modality == mapl.commit:
                    new_var_relaxations |= set(svar.args)
                    new_value_relaxations |= set(svar.modal_args)
                elif supporter[svar] != self.plan.init_node:
                    new_var_relaxations |= set(svar.args)
                    new_value_relaxations.add(val)
                elif svar.function.type != pddl.t_number:
                    for a in svar.args+svar.modal_args:
                        new_constraints[a].add(state.Fact(svar,val))
            new_var_relaxations.discard(pddl.TRUE)
            new_var_relaxations.discard(pddl.FALSE)
            new_value_relaxations.discard(pddl.TRUE)
            new_value_relaxations.discard(pddl.FALSE)

            new_free_vars = set()
            new_free_values = set()
            for svar in var_relaxations|value_relaxations:
                log.debug("trying to relax %s ...", str(svar))
                if svar in new_var_relaxations | new_value_relaxations:
                    log.debug("will be relaxed later")
                    continue
                if svar in new_constraints:
                    log.debug("has some new constraints: %s", str(map(str, new_constraints[svar])))
                    new_fixed.discard(svar)
                    continue
                log.debug("is completely free")
                if svar in var_relaxations:
                    new_free_vars.add(svar)
                else:
                    new_free_values.add(svar)

            for remove in [None]+list(new_free_values) + list(new_free_vars):
                combined_constraints = defaultdict(set)
                combined_constraints.update(new_constraints)
                if remove:
                    log.debug("removing: %s", str(remove))
                    new_fixed.discard(remove)
                if new_fixed >= fixed and fixed:
                    continue

                for obj, constr in constraints.iteritems():
                    for c in constr:
                        if c.value in new_fixed:
                            combined_constraints[obj].add(c)
                        else:
                            log.debug("%s: constraint %s is gone.", obj.name, str(c))
                        
                log.debug("fixed on this layer: %s", str(map(str, new_fixed)))
                c2 = transform_constraints(new_fixed, combined_constraints)
                fc = partial_problem.ObjectsConstraint(new_fixed)
                layers.append([fc]+c2)
            #print "fixed values:", map(str, fixed)
            #print "possible relaxations:", map(str, relaxations)
            log.debug("")
            fixed = new_fixed
            var_relaxations = new_var_relaxations
            value_relaxations = new_value_relaxations
            constraints = combined_constraints

        for i, constraints in enumerate(layers):
            log.debug("Layer %d", i)
            log.debug("Fixed: %s", str(constraints[0]))
            # cset = set()
            # for c in constraints.itervalues():
            #     cset |= c
            log.debug("Constraints: %s", str(map(str, constraints[1:])))
            log.debug("")
            
        return layers
            
    def create_goal_actions(self, goals, domain):
        commit_actions = []

        confirm_score = 100
        
        for svar in goals:
            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args())
            domain.add(svar.get_args())
            
            val = pddl.Parameter("?val", svar.function.type)
            name = "commit-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            a = pddl.Action(name, [val], None, None, domain)
            b = pddl.builder.Builder(a)
            
            #a.precondition = b.cond('and', ('not', (dtpddl.committed, [term])))
            a.precondition = b.cond('not', ('done', ))
            #commit_effect = b.effect(dtpddl.committed, [term])
            reward_effect = b('when', ('=', term, val), ('assign', ('reward',), confirm_score))
            penalty_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), -confirm_score))
            done_effect = b.effect('done')
            a.effect = b.effect('and', reward_effect, penalty_effect, done_effect)
            
            commit_actions.append(a)

        disconfirm = []
        for pnode in self.select_actions:
            for svar, val in pnode.effects:
                if svar.modality == mapl.commit:
                    nmvar = svar.nonmodal()
                    if nmvar not in goals:
                        disconfirm.append((nmvar, svar.modal_args[0]))

        if not disconfirm:
            return commit_actions
                        
        dis_score = float(confirm_score)/len(disconfirm)
        disconfirm_actions = []
        for svar, val in disconfirm:
            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args() + [val])
            domain.add(svar.get_args() + [val])
            
            name = "disconfirm-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            a = pddl.Action(name, [], None, None, domain)
            b = pddl.builder.Builder(a)

            #conds = [b.cond('not', (dtpddl.committed, term))]
            conds = [b.cond('not', ('done',))]
            # for gvar in goals:
            #     # don't allow this goal after a commit has been done
            #     gterm = pddl.Term(gvar.function, gvar.get_args())
            #     conds.append(b.cond('not', (dtpddl.committed, gterm)))
            # for ca in commit_actions:
            #     # don't allow commit actions after any disconfirm
            #     ca.precondition.parts.append(b.cond('not', (dtpddl.committed, term)))
            
            #a.precondition = pddl.Conjunction(conds, a)
            a.precondition = b.cond('not', ('done', ))
            
            #commit_effect = b.effect(dtpddl.committed, term)
            reward_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), dis_score))
            penalty_effect = b('when', ('=', term, val), ('assign', ('reward',), -2*dis_score))
            done_effect = b.effect('done')
            a.effect = b.effect('and', reward_effect, penalty_effect, done_effect)
            
            disconfirm_actions.append(a)
                        
        return commit_actions + disconfirm_actions

    def create_dt_domain(self, dom):
        dtdomain = dom.copy()
        dtdomain.name = "dt-%s" % dom.name
        dtdomain.annotations =  pddl.translators.Translator.get_annotations(dom)
        return dtdomain

    def recompute_problem(self, new_state):
        self.state = new_state
        self.subproblems = self.compute_subproblems(self.state)
        self.problem = self.create_problem(self.state, self.dtdomain)

    def compute_subproblems(self, cast_state):
        # import debug
        # debug.set_trace()
        
        prob_by_layer = {}
        problems =  []
        all_objects = cast_state.objects | cast_state.generated_objects | self.domain.constants
        problems.append(partial_problem.PartialProblem(cast_state.prob_state, all_objects,  [], [], self.dt_rules, self.domain))
        i = len(self.relaxation_layers)-1
        prev_constraints = []
        for constraints in reversed(self.relaxation_layers):
            # Iterate over relaxation layers first (from most to least relaxed)
            log.debug("Next layer %d:", i)
            problems.append(partial_problem.PartialProblem(cast_state.prob_state, problems[-1].objects, prev_constraints, constraints, self.dt_rules, self.domain))
                    
            i -= 1
            prev_constraints = constraints

        return problems
        

    def create_problem(self, cast_state, domain):
        t0 = time.time()
        opt = "maximize"
        opt_func = pddl.FunctionTerm(pddl.dtpddl.reward, [])

        selected = 0
        for i, prob in enumerate(self.subproblems):
            selected = i
            #log.debug("Problem %i: approx. State size: %d - %d", i, prob.size, prob.max_size)
            log.debug("Problem %i: %s", i, str(prob))
            if prob.min_size < global_vars.config.dt.max_state_size:
                log.info("Selecting layer %d with minimal approx. state size of %d", i, prob.min_size)
                break

        base_problem = self.subproblems[0]
        problem = self.subproblems[selected]
        objects = problem.objects
        facts = problem.facts

        self.selected_subproblem = selected

        trees = StateTreeNode.create_root(cast_state.prob_state, objects, self.dt_rules)
        hstate = HierarchicalState([], cast_state.prob_state.problem)
        for t in trees:
            t.create_state(hstate)
            
        #reduce the problem some more if neccessary
        while problem.max_size > global_vars.config.dt.max_state_size:
            if not problem.reduce(hstate):
                assert False
        
        #facts = [f.to_init() for f in cast_state.prob_state.iterdists()]
        p_facts = hstate.init_facts()
        p_objects = set(o for o in problem.objects if o not in domain.constants)

        # print map(str, p_objects)
        # print map(str, p_facts)
        problem = pddl.Problem("cogxtask", p_objects, p_facts, None, domain, opt, opt_func )
        problem.goal = pddl.Conjunction([])
        log.debug("total time for state creation: %f", time.time()-t0)
        return problem

    def create_prob_state(self, rules, state):
        pass
    
    def find_observation_actions(self):
        def can_observe(action):
            for observe in self.domain.observe:
                if not observe.execution:
                    return True
                for ex in observe.execution:
                    if ex.negated:
                        return ex.action != action
                    if ex.action == action:
                        return ex
                return False

        return [a for a in self.domain.actions if can_observe(a)]
            
    def get_observe_action_preconditions(self):
        all_prec = set()
        for a in self.find_observation_actions():
            all_prec |= set(visitors.visit(a.precondition, function_visitor, []))
        return all_prec
        
class DTPDDLOutput(task.PDDLOutput):
    def __init__(self):
        self.compiler = pddl.translators.ChainingTranslator(dtpddl.DTPDDLCompiler(), dtpddl.ProbADLCompiler())
        self.writer = dtpddl.DTPDDLWriter()
        self.supported = task.adl_support + ['action-costs', 'partial-observability', 'fluents', 'mapl']

@visitors.collect
def function_visitor(elem, result):
    if isinstance(elem, pddl.FunctionTerm):
        if not elem.function.builtin:
            return sum(result, []) + [elem.function]
    if isinstance(elem, pddl.Literal):
        result = []
        if not elem.predicate.builtin:
            result = [elem.predicate]
        else:
            return result + sum([t.visit(function_visitor) for t in elem.args], [])
        return result

class StateTreeNode(dict):
    def __init__(self, rule, st, objects, all_rules, cache, fact=None):
        self.rule = rule
        self.state = st
        self.all_rules = all_rules
        self.objects = objects
            
        self.cache = cache
        
        if fact is not None:
            self.svar = fact.svar
            rules_for_fact = [r for r in all_rules if r.function == self.svar.function]
            if rules_for_fact:
                self.rule = rules_for_fact[0]
            else:
                self.rule = None
                
            if isinstance(fact.value, pddl.TypedObject):
                self.create_subtree(1.0, fact.value, marginal=(val not in objects))
            else:
                for val, p in fact.value.iteritems():
                    self.create_subtree(p, val, marginal=(val not in objects))
            return
            
        #assume that the rule is instantiated
        self.svar = state.StateVariable(rule.function, state.instantiate_args(rule.args))
        
        log.debug("creating subtree for %s", str(self.svar))
        for p, value in rule.values:
            pval = self.state.evaluate_term(p) # evaluate in original state because probs for marginals could be gone in this partial state
            if pval == pddl.UNKNOWN or pval.value < 0.01:
                continue
            val = self.state.evaluate_term(value)
            self.create_subtree(pval.value, val, marginal=(val not in objects))

    def create_state(self, st):
        for val, (subtrees, p, marginal) in self.iteritems():
            if p == 1.0:
                sub = st
            else:
                sub = HierarchicalState([], parent=st)
                st.add_substate(self.svar, val, sub, p)
            if not marginal:
                sub[self.svar] = val
            for t in subtrees:
                t.create_state(sub)


    def create_subtree(self, prob, value, marginal=False):
        log.debug("creating subtrees for %s=%s (marginal=%s)", str(self.svar), value, str(marginal))
        fact = state.Fact(self.svar, value)
        if fact in self.cache:
            print "cache hit"
            self[value] = (self.cache[fact], prob, marginal)
            return

        if self.rule is None:
            self[value] = ([], prob, marginal)
            return

        rules = [r for r in self.all_rules if r.depends_on(self.rule)]
        log.debug("rules depending on this: %s", ", ".join(r.function.name for r in rules))
        
        subtrees = []
        for r in rules:
            matched_cond = None
            matched_args = {}
            for t,v in r.conditions:
                if t.function == self.rule.function:
                    matched_cond = t
                    for a, a2 in zip(t.args + [v], self.svar.args + (value,)):
                        matched_args[a.object] = a2
                    break
            if matched_cond and any(a.__class__ == a2.__class__ == pddl.TypedObject and a != a2 for a,a2 in matched_args.iteritems()):
                continue

            def get_objects(arg):
                if arg in matched_args:
                    return [matched_args[arg]]
                if arg in r.get_value_args():
                    return list(self.state.problem.get_all_objects(arg.type))
                return [o for o in self.objects if o.is_instance_of(arg.type)]


            args = r.args+r.add_args
            for mapping in r.smart_instantiate(r.get_inst_func(self.state, matched_cond), args, [get_objects(a) for a in args], self.state.problem):
                subtrees.append(StateTreeNode(r, self.state, self.objects, self.all_rules, self.cache))
        
        self.cache[fact] = subtrees
        self[value] = (subtrees, prob, marginal)

    @staticmethod
    def create_from_fact(fact, st, objects, all_rules, cache):
        return StateTreeNode(None, st, objects, all_rules, cache, fact=fact)

    @staticmethod
    def create_root(st, objects, rules, cache=None):
        nodep_rules = []
        for r in rules:
            if all(not r.depends_on(r2) for r2 in rules):
                nodep_rules.append(r)

        if cache is None:
            cache = {}

        subtrees = []
        done = set()
        for fact in st.iterdists():
            if any(a not in objects for a in fact.svar.args):
                continue
            sub = StateTreeNode.create_from_fact(fact, st, objects, rules, cache)
            if sub is not None:
                subtrees.append(sub)
                done.add(sub.svar)
                
        for r in nodep_rules:
            def get_objects(arg):
                if arg in r.get_value_args():
                    return list(st.problem.get_all_objects(arg.type))
                return [o for o in objects if o.is_instance_of(arg.type)]
            
            args = r.args+r.add_args
            for mapping in r.smart_instantiate(r.get_inst_func(st), args, [get_objects(a) for a in args], st.problem):
                svar = state.StateVariable(r.function, state.instantiate_args(r.args))
                if svar in done:
                    continue
                subtrees.append(StateTreeNode(r, st, objects, rules, cache))

        return subtrees
        

class HierarchicalState(state.State):
    def __init__(self, facts=[], prob=None, parent=None):
        """Create a new State object.

        Arguments:
        facts -- List of Fact objects or (StateVariable, TypedObject) tuples the State should be initialized with.
        prob -- The PDDL problem description this State is based on.
        parent -- The parent of this state."""

        self.parent = parent
        if parent and prob is None:
            self.problem = parent.problem
        else:
            self.problem = prob
            
        for f in facts:
            self.set(f)

        self.hash = hash(tuple(facts) + (parent, ))

        self.read_svars = set()
        self.written_svars = set()
        
        self.substates = defaultdict(dict)

    def get_joint_prob(self, svars):
        detvars = {}
        for svar in svars:
            if svar in self and svar != pddl.UNKNOWN:
                detvars[svar] = self[svar]
        remaining = [svar for svar in svars if svar not in detvars]
        sub_result = {(pddl.UNKNOWN,)*len(remaining) : 1.0}
        for d in self.substates.itervalues():
            res = defaultdict(lambda: 0.0)
            for p, state in d.itervalues():
                for vals, p2 in state.get_joint_prob(remaining):
                    if p2 > 0.0 and any(v != pddl.UNKNOWN for v in vals):
                        res[vals] += p*p2
            if res:
                sub_result = res
                break
        
        result = []
        for vals, p in sub_result.iteritems():
            entry = []
            for svar in svars:
                if svar in detvars:
                    entry.append(detvars[svar])
                else:
                    entry.append(vals[remaining.index(svar)])
            result.append((tuple(entry), p))
        return result
        

    def get_prob(self, svar, value=None):
        if svar in self and svar != pddl.UNKNOWN:
            if value is None:
                return [(self[svar], 1.0)]
            elif self[svar] == value:
                return 1.0
            else:
                return 0.0
        for d in self.substates.itervalues():
            if value is None:
                res = defaultdict(lambda: 0.0)
                for p, state in d.itervalues():
                    for val, p2 in state.get_prob(svar):
                        if p2 > 0.0:
                            res[val] += p*p2
                if res:
                    return list(d.iteritems())
            else:
                p_total = 0.0
                for p, state in d.itervalues():
                    p_total += p * state.get_prob(svar, value)
                if p_total > 0.0:
                    return p_total
        if value is None:
            return []
        return 0.0

    def size(self):
        size = 1
        for d in self.substates.itervalues():
            subsize = 0
            p_total = 0
            for p, sub in d.itervalues():
                p_total += p
                subsize += sub.size()
            if p_total < 0.99:
                subsize += 1
            size *= subsize
        return size

    def is_empty(self):
        if any(val != pddl.UNKNOWN for val in self.itervalues()):
            return False
        for d in self.substates.itervalues():
            if any(not sub.is_empty() for p,sub in d.itervalues()):
                return False
        return True

    def cleanup(self):
        to_delete = []
        for svar, d in self.substates.iteritems():
            if all(sub.is_empty() for p, sub in d.itervalues()):
                to_delete.append(svar)
            for p, sub in d.itervalues():
                sub.cleanup()
        for svar in to_delete:
            del self.substates[svar]

    def add_substate(self, svar, val, state, prob):
        self.substates[svar][val] = (prob, state)

    def get_substate(self, svar, val):
        return self.substates[svar][val]

    def delete_substate(self, svar, val):
        del self.substates[svar][val]

    def has_substate(self, svar, val=None):
        if val is None:
            return svar in self.substates and self.substates[svar]
        return svar in self.substates and val in self.substates[svar]
    
    def get_substates(self, svar):
        return self.substates[svar]

    def init_facts(self):
        elems = []
        for f in self.iterfacts():
            if f.value != pddl.UNKNOWN:
                eff = f.as_literal(_class=effects.SimpleEffect)
                if eff.predicate == pddl.builtin.assign:
                    eff.predicate = pddl.builtin.equal_assign
                if eff.predicate == pddl.builtin.num_assign:
                    eff.predicate = pddl.builtin.num_equal_assign
                elems.append(eff)
        for varsubs in self.substates.itervalues():
            tups = []
            for p, sub in varsubs.itervalues():
                tups.append((pddl.Term(p), sub.to_effect()))
            elems.append(effects.ProbabilisticEffect(tups))
            
        return elems
    
    def to_effect(self):
        elems = []
        for f in self.iterfacts():
            if f.value != pddl.UNKNOWN:
                elems.append(f.as_literal(_class=effects.SimpleEffect))
        for varsubs in self.substates.itervalues():
            tups = []
            for p, sub in varsubs.itervalues():
                tups.append((pddl.Term(p), sub.to_effect()))
            elems.append(effects.ProbabilisticEffect(tups))
            
        if len(elems) == 1:
            return elems[0]
        return effects.ConjunctiveEffect(elems)

    def __hash__(self):
        return self.hash

    def __eq__(self, o):
        return o.__class__  == self.__class__ and o.hash == self.hash
    
    def __getitem__(self, key):
        try:
            return dict.__getitem__(self, key)
        except:
            if self.parent:
                return self.parent[key]
            if isinstance(key.function, pddl.Predicate):
                return pddl.FALSE
            return pddl.UNKNOWN
        
    def __setitem__(self, svar, value):
        assert isinstance(svar, state.StateVariable)
        if isinstance(value, (float, int, long)):
            value = pddl.TypedNumber(value)
        assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        dict.__setitem__(self, svar, value)

    def __contains__(self, key):
        if isinstance(key, state.Fact):
            return key.svar in self and self[key.svar] == key.value
        if not dict.__contains__(self, key):
            if self.parent:
                return key in self.parent
            return False
        return True
