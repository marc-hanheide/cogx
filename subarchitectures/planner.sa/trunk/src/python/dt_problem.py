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
            for pred in plan.predecessors_iter(pnode, 'depends'):
                restr = find_restrictions(pred)
                if pred.action.name.startswith("select-"):
                    return [pred] + restr
                if restr:
                    return restr
            return []

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
            new_fixed = fixed | set(pnode.full_args)
            new_var_relaxations = set()
            new_value_relaxations = set()
            new_constraints = defaultdict(set)
            for svar, val in pnode.preconds:
                if svar.modality == mapl.commit:
                    new_var_relaxations |= set(svar.args)
                    new_value_relaxations |= set(svar.modal_args)
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
            
            a.precondition = b.cond('and', ('not', (dtpddl.committed, [term])))
            commit_effect = b.effect(dtpddl.committed, [term])
            reward_effect = b('when', ('=', term, val), ('assign', ('reward',), confirm_score))
            penalty_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), -2*confirm_score))
            a.effect = b.effect('and', commit_effect, reward_effect, penalty_effect)
            
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

            conds = [b.cond('not', (dtpddl.committed, term))]
            for gvar in goals:
                # don't allow this goal after a commit has been done
                gterm = pddl.Term(gvar.function, gvar.get_args())
                conds.append(b.cond('not', (dtpddl.committed, gterm)))
            for ca in commit_actions:
                # don't allow commit actions after any disconfirm
                ca.precondition.parts.append(b.cond('not', (dtpddl.committed, term)))
            
            a.precondition = pddl.Conjunction(conds, a)
            
            commit_effect = b.effect(dtpddl.committed, term)
            reward_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), dis_score))
            penalty_effect = b('when', ('=', term, val), ('assign', ('reward',), -10*dis_score))
            a.effect = b.effect('and', commit_effect, reward_effect, penalty_effect)
            
            disconfirm_actions.append(a)
                        
        return commit_actions + disconfirm_actions

    def create_dt_domain(self, dom):
        dtdomain = dom.copy()
        dtdomain.name = "dt-%s" % dom.name
        dtdomain.annotations =  pddl.translators.Translator.get_annotations(dom)
        return dtdomain
    
    # def create_limited_domain(self, dom):
    #     dtdomain = dom.copy()
    #     dtdomain.name = "dt-%s" % dom.name
    #     all_actions = dtdomain.actions
    #     dtdomain.actions = []
    #     dtdomain.axioms = []

    #     observe_preconds = self.get_observe_action_preconditions()
    #     for a in all_actions:
    #         effects = set(visitors.visit(a.effect, function_visitor, []))
    #         if not observe_preconds & effects:
    #             dtdomain.actions.append(a)

    #     return dtdomain


    def compute_removal_order(self, to_remove, fixed, st):
        OT_VALUE = 0 
        OT_CORRELATED = 1 

        types = set(o.type for o in to_remove)
        fixed_types = set(o.type for o in fixed)

        # the value to remove occurs as a value of an already fixed feature
        value_order_rules = defaultdict(set)
        # the value to remove occurs as a parameter together with an already fixed feature
        corr_order_rules = defaultdict(set)
        for t in types:
            for r in self.dt_rules:
                for _,v in r.values:
                    if t.equal_or_subtype_of(v.get_type()):
                        value_order_rules[t].add(r)
                        break
                for a in r.args:
                    if t.equal_or_subtype_of(a.type):
                        if any(ft.equal_or_subtype_of(a2.type) for ft in fixed_types for a2 in r.args if a2 != a):
                            corr_order_rules[t].add(r)
                            break
        sorted_by_type = dict((t, [o for o in to_remove if o.type == t]) for t in types)
        for t, rules in value_order_rules.iteritems():
            log.debug("ordering %s by probability", str(t))
            obj_by_prob = []
            for o in to_remove:
                if t != o.type:
                    continue
                p_total = 0
                p_count = 0
                for r in rules:
                    combinations = state.product(*map(lambda a: [o for o in fixed if o.is_instance_of(a.type)], r.args))
                    for c in combinations:
                        p = r.get_probability(c, o)
                        pval = st.evaluate_term(p)
                        if pval != UNKNOWN:
                            p_total += p.value
                            p_count +=1
                p_avg = p_total/p_count
                obj_by_prob.append((o,p_avg))
            sorted_by_type[t] = sorted(obj_by_prob, key=lambda (o,p): p)

        for t, rules in corr_order_rules.iteritems():
            log.debug("ordering %s by disambiguation power", str(t))
            obj_by_prob = []
            for o in to_remove:
                if t != o.type:
                    continue
                p_total = 0
                p_count = 0
                for r in rules:
                    combinations = state.product(*map(lambda a: [o for o in fixed if o.is_instance_of(a.type)], r.args))
                    for c in combinations:
                        p = r.get_probability(c, o)
                        pval = st.evaluate_term(p)
                        if pval != UNKNOWN:
                            p_total += p.value
                            p_count +=1
                p_avg = p_total/p_count
                obj_by_prob.append((o,p_avg))
            sorted_by_type[t] = sorted(obj_by_prob, key=lambda (o,p): p)
            

        for t, rules in corr_order_rules.iteritems():
            log.debug("%ss should be ordered by disambiguation power", str(t))
                
        return list(to_remove)
        

    def compute_subproblems(self, cast_state):
        # import debug
        # debug.set_trace()
        
        prob_by_layer = {}
        problems =  []
        all_objects = cast_state.objects | cast_state.generated_objects | self.domain.constants
        problems.append(partial_problem.PartialProblem(cast_state.prob_state, all_objects,  [], [], self.dt_rules, self.domain))
        i = len(self.relaxation_layers)-1
        #j = 1
        #prev_fixed = set()
        prev_constraints = []
        for constraints in reversed(self.relaxation_layers):
            # Iterate over relaxation layers first (from most to least relaxed)
            log.debug("Next layer %d:", i)
            problems.append(partial_problem.PartialProblem(cast_state.prob_state, problems[-1].objects, prev_constraints, constraints, self.dt_rules, self.domain))
            # new_fixed = fixed - prev_fixed
            # fixed_types = set(o.type for o in new_fixed)
            # #log.debug("New fixed: %s", ", ".join(o.name for o in new_fixed))

            # generic_constraints = []
            # for t,v in constraints:
            #     if v in new_fixed:
            #         generic_constraints.append((t,v))

            # # log.debug("Generic constraints: %s", ", ".join("%s = %s" % (str(t), v.name) for t,v in generic_constraints))
            # alt_objects = set(o for o in problems[-1].objects if o.type in fixed_types and o not in fixed)
            # #to_remove = self.compute_removal_order(to_remove, fixed, cast_state.prob_state)
            
            # log.debug("Removing the following objects: %s", ", ".join(o.name for o in alt_objects))
            # objects, facts = self.limit_state(cast_state, fixed | set(alt_objects), constraints, problems[-1].objects)
            # problems.append(PartialProblem(objects - alt_objects, alt_objects, facts, self.dt_rules, self.domain))
            # while to_remove:
            #     #compute possible intermediate problems between two layers
            #     rem = to_remove.pop(0)
            #     new_constraints = []
            #     for t,v in generic_constraints:
            #         c2 = []
            #         new_constraints.append(c2)
            #         c2.append(state.Fact(t,v))
            #         for o in to_remove:
            #             if o.type == v.type:
            #                 c2.append(state.Fact(t,o))
            #     objects, facts = self.limit_state(cast_state, fixed | set(to_remove), new_constraints, problems[-1].objects)
            #     log.debug("problem %d: %s", j, ", ".join(o.name for o in objects))
            #     problems.append(PartialProblem(objects, facts, self.dt_rules, self.domain))
            #     j += 1
                    
            i -= 1
            #prev_fixed = fixed
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
        
        #
        # Build dependencies for the rules
        #
        prob_functions = set(r.function for r in self.dt_rules)
        ruledeps = {}
        nondep_rules = []
        
        for r in self.dt_rules:
            r.set_parent(cast_state.prob_state.problem)
            deps = False
            prob_deps = r.deps() & prob_functions 
            assert len(prob_deps) <= 1 # should be enough for now
            ruledeps[r] = prob_deps
            if not prob_deps:
                nondep_rules.append(r)

        substates = defaultdict(list)

        hstate = HierarchicalState([], cast_state.prob_state.problem)
        for f in facts:
            detval = None
            for val, prob in f.value.iteritems():
                if prob == 1.0:
                    detval = val
                    break
            if detval:
                hstate[f.svar] = detval
                continue
            
            for val, prob in f.value.iteritems():
                if prob >= 0.99:
                    hstate[f.svar] = val
                if prob > 0.0 and val != pddl.UNKNOWN:
                    substate = HierarchicalState([state.Fact(f.svar,val)], parent=hstate)
                    hstate.add_substate(f.svar, val, substate, prob)
                    substates[f.svar.function].append(substate)
                    log.debug("create substate for %s = %s (p=%.2f)", str(f.svar), str(val), prob)

        marginal_substates = []
        marginal_objects = defaultdict(set)
        
        for f in base_problem.facts:
            if hstate.has_substate(f.svar):
                for val, prob in f.value.iteritems():
                    if prob > 0.0 and not hstate.has_substate(f.svar, val):
                        substate = HierarchicalState([state.Fact(f.svar,val)], parent=hstate)
                        hstate.add_substate(f.svar, val, substate, prob)
                        substates[f.svar.function].append(substate)
                        log.debug("create marginal substate for %s = %s (p=%.2f)", str(f.svar), str(val), prob)
                        marginal_substates.append((hstate, f.svar, val))
                        marginal_objects[val.type].add(val)

        def apply_rule(st, r, pred, marginal=False):
            svar = state.StateVariable(r.function, state.instantiate_args(r.args))
            if svar in st:
                return

            nondet_conditions = []
            for t,v in r.conditions:
                if t.function in prob_functions:
                    nondet_conditions.append((t,v))
                    continue # check those later
                cvar = state.StateVariable(t.function, state.instantiate_args(t.args))
                val = hstate.evaluate_term(v)
                if hstate[cvar] != val:
                    #print "not satisfied in top level state: %s = %s" % (str(cvar), str(val))
                    return
            if not pred:
                rel_substates = [st]
            else:
                rel_substates = substates[pred]
            for st2 in rel_substates:
                sat = True
                #print "state:", st2
                for t,v in nondet_conditions:
                    cvar = state.StateVariable(t.function, state.instantiate_args(t.args))
                    val = st2.evaluate_term(v)
                    if st2[cvar] != val:
                        #print "not satisfied: %s = %s" % (str(cvar), str(val))
                        sat = False
                        break
                if not sat:
                    continue
                
                for p,v in r.values:
                    pval = cast_state.prob_state.evaluate_term(p) # evaluate in original state because probs for marginals could be gone in this partial state
                    if pval == pddl.UNKNOWN or pval.value < 0.01:
                        continue
                    val = st2.evaluate_term(v)
                    if svar in st2 or st2.has_substate(svar, val):
                        continue
                    if pval.value > 0.99:
                        st2[svar] = val
                        continue
                    sub = HierarchicalState([state.Fact(svar, val)], parent=st2)
                    st2.add_substate(svar, val, sub, pval.value)
                    substates[svar.function].append(sub)
                    log.debug("create substate for %s = %s (p=%.2f)", str(svar), str(val), pval.value)
                    if marginal:
                        log.debug("This is a marginal state")
                        marginal_substates.append((st2, svar, val))
                        marginal_objects[val.type].add(val)
            
                    
        closed = set()
        closed_functions = set()
        open = set(nondep_rules)
        while open:
            r = open.pop()
            log.debug("Rule: %s", r.function.name)
            if ruledeps[r]:
                pred = iter(ruledeps[r]).next()
            else:
                pred = None

            #print marginal_objects
            combinations = state.product(*map(lambda a: problem.get_objects(a.type) | marginal_objects[a.type], r.args+r.add_args))
            for c in combinations:
                #print "(%s %s)" % (r.function.name, " ".join(a.name for a in c))
                r.instantiate_all(c, cast_state.prob_state.problem)
                apply_rule(hstate, r, pred)
                r.uninstantiate()

            value_args = r.get_value_args()
            if value_args:
                def get_values_for_marginals(arg):
                    if arg in value_args:
                        return base_problem.get_objects(arg.type) - problem.get_objects(arg.type)
                    return problem.get_objects(arg.type)
                
                combinations = state.product(*map(lambda a: get_values_for_marginals(a), r.args+r.add_args))
                for c in combinations:
                    #print "marginal: (%s %s)" % (r.function.name, " ".join(a.name for a in c))
                    r.instantiate_all(c, cast_state.prob_state.problem)
                    apply_rule(hstate, r, pred, marginal=True)
                    r.uninstantiate()

            closed.add(r)
            closed_functions.add(r.function)
            
            for r in self.dt_rules:
                #print r, map(str, (r.deps() & prob_functions) - done)
                if r not in open and r not in closed and not (ruledeps[r] - closed_functions):
                    open.add(r)

        marginals_per_svar = defaultdict(set)
        for st, svar, val in marginal_substates:
            marginals_per_svar[st, svar].add(val)

        for (st, svar), vals in marginals_per_svar.iteritems():
            for v in vals:
                p, sub = st.get_substate(svar, v)
                if svar in sub:
                    del sub[svar]

        #reduce the problem some more if neccessary
        while problem.max_size > global_vars.config.dt.max_state_size:
            if not problem.reduce(hstate):
                assert False
        
        #facts = [f.to_init() for f in cast_state.prob_state.iterdists()]
        p_facts = hstate.init_facts()
        p_objects = set(o for o in objects if o not in domain.constants)

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
