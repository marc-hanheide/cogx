import time, math, itertools
from itertools import chain, product

from collections import defaultdict
from standalone import task, config, pddl, plans
from standalone.pddl import state, dtpddl, mapl, translators, visitors, effects

import standalone.globals as global_vars
import simplegraph
import partial_problem

log = config.logger("dt")


class DTProblem(object):
    def __init__(self, plan, pnodes, failed_goals, prob_functions, domain):
        self.plan = plan
        self.pnodes = pnodes
        self.domain = domain
        self.prob_functions = prob_functions
        #self.state = cast_state
        self.subplan_actions = []
        self.dt_plan = []
        self.select_actions = []
        
        self.goals = self.create_goals(plan)
        if not self.goals:
            return
        
        self.dt_rules = pddl.translators.Translator.get_annotations(domain).get('dt_rules', [])
        
        self.dtdomain = self.create_dt_domain(domain)
        self.goal_actions = self.create_goal_actions(self.goals, failed_goals, self.dtdomain)
        self.dtdomain.actions += [a for a in self.goal_actions]
        self.dtdomain.name2action = None

        # dom_str, prob_str = DTPDDLOutput().write(self.problem)
        # print "\n".join(dom_str)
        # print "\n".join(prob_str)
        
    def initialize(self, prob_state):
        self.state = prob_state
        #facts = dtpddl.PNode.reduce_all(self.pnodes, choices, global_vars.config.dt.max_state_size)
        facts = self.reduce_state(global_vars.config.dt.max_state_size)
        selected = set()
        add_objects = set()
        for var, vals in facts.iteritems():
            add_objects.update(var.args)
            add_objects.update(var.modal_args)
            for v in vals:
                add_objects.add(v)
                selected.add(state.Fact(var, v))
                log.debug("selected: %s, %s", var, v)

        def objects(svar, val):
            return set(svar.args + svar.modal_args + (val,))
                
        def get_used_objects(node):
            used = set()
            pruned = set()
            relevant = False
            for val, (p, nodes, facts) in node.children.iteritems():
                branch_rel = state.Fact(node.svar, val) in selected
                branch_used = set()
                branch_pruned = set()
                for svar, val in facts.iteritems():
                    f = state.Fact(svar, val)
                    branch_used |= objects(svar, val)
                    branch_rel |= (f in selected)
                    # print "rel:", branch_rel, f
                for n in nodes:
                    nused, npruned, nrel = get_used_objects(n)
                    branch_used |= nused
                    branch_pruned |= npruned
                    branch_rel |= nrel
                relevant |= branch_rel
                pruned |= branch_pruned
                if branch_rel:
                    used |= branch_used
                else:
                    pruned |= branch_used
            pruned -= used
            if used:
                # print map(str, used)
                assert relevant

            return used, pruned, relevant

        pruned_objects = set()
        used_objects = set()
        for used, pruned, rel in map(get_used_objects, self.pnodes):
            pruned_objects |= pruned
            used_objects |= used
        pruned_objects -= used_objects

        # unused_objects = set()
        # for svar, val in chain(*map(all_facts, self.pnodes)):
        #     for obj in objects(svar, val):
        #         if obj not in self.domain:
        #             unused_objects.add(obj)

        # for svar, vals in facts.iteritems():
        #     for obj in chain(svar.args, svar.modal_args, vals):
        #         unused_objects.discard(obj)

        new_objects = (self.state.problem.objects | add_objects) - pruned_objects

        # print map(str, selected)

        opt = "maximize"
        opt_func = pddl.FunctionTerm(pddl.dtpddl.reward, [])
        init = [f.to_init() for f in self.state.deterministic() if not (objects(*f) & pruned_objects)]
        for n in self.pnodes:
            peff = n.to_init(selected)
            if peff:
                init.append(peff)
            
        self.problem = pddl.Problem("cogxtask", new_objects, init, None, self.dtdomain, opt, opt_func)
        self.problem.goal = pddl.Conjunction([])

        #self.selected_subproblem = -1

        #for pnode in self.subplan_actions:
        #    pnode.status = plans.ActionStatusEnum.IN_PROGRESS
        
        #self.relaxation_layers = self.compute_restrictions_new()
        #self.relaxation_layers = self.compute_restrictions()
        #self.subproblems = self.compute_subproblems(self.state)
        #self.problem = self.create_problem(self.state, self.dtdomain)

    def extract_choices(self):
        choices = {}
        for pnode, level in self.select_actions:
            for fact in pnode.effects:
                if not fact.svar.modality and fact.svar.get_type().equal_or_subtype_of(pddl.t_object):
                    choices[fact.svar] = fact.value
        return choices

    def write_dt_input(self, domain_fn, problem_fn):
        DTPDDLOutput().write(self.problem, domain_fn=domain_fn, problem_fn=problem_fn)

    def observation_expected(self, action):
        for o in self.domain.observe:
            for e in o.execution:
                if e.action.name == action.name:
                    return True
        return False

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

        def find_restrictions(pnode, level):
            others = []
            for pred in plan.predecessors_iter(pnode, 'depends'):
                restr = find_restrictions(pred, level+1)
                if pred.action.name.startswith("commit-"):
                    return [(pred, level+1)] + restr
                if restr:
                    others = restr
            return others

        self.select_actions = []
        goal_facts = set()
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            if pnode.action.name in observe_actions:
                #print pnode.action.name
                #print map(str, pnode.effects)
                #TODO: only add an action if the observe effect supports a later action
                for fact in pnode.effects:
                    if fact.svar.function not in (pddl.builtin.total_cost, ):
                        goal_facts.add(fact)
                    # if svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                    #     goal_svars.add(svar.nonmodal())
                self.subplan_actions.append(pnode)
                self.select_actions += [(pnode,0)] + find_restrictions(pnode,0)
                #break # only one action at a time for now
            
            if pnode.action.name not in observe_actions and self.subplan_actions:
                break
            
        return goal_facts
    

    def replanning_neccessary(self, new_state):
        #if self.selected_subproblem == -1:
        #    return True
        #new_objects = new_state.problem.objects - self.state.problem.objects
        #problem = self.subproblems[self.selected_subproblem]
        #for o in new_objects:
        #    if all(c.matches(o, self.state) for c in problem.constraints):
        #        return True
        return False

    def compute_restrictions_new(self):
        def transform_constraints(new_fixed, new_constraints):
            #print map(str, new_fixed)
            c2 = []
            most_relaxed_supertype = {}
            for c in new_constraints:
                for a, p in zip(c.svar.args, c.svar.function.args):
                    if a not in new_fixed:
                        if not a in most_relaxed_supertype:
                            most_relaxed_supertype[a] = p.type
                        elif p.type.is_subtype_of(most_relaxed_supertype[a]):
                            most_relaxed_supertype[a] = p.type
                            
            replace_dict = {}
            for c in new_constraints:
                args = []
                assert not c.svar.modal_args, "Not yet supported."
                for a in c.svar.args:
                    if a in new_fixed:
                        args.append(a)
                    else:
                        typ = most_relaxed_supertype[a]
                        a2 = pddl.TypedObject("any_%s" % (typ.name), typ)
                        replace_dict[a] = a2
                        args.append(a2)
                val = replace_dict.get(c.value, c.value)
                constr = partial_problem.FunctionConstraint(c.svar.function, args, [val])
                if constr not in c2:
                    c2.append(constr)
            return c2

        fixed_per_pnode = {}
        fixed = set()
        for pnode in reversed(self.select_actions):
            fixed = fixed | set(pnode.full_args)
            fixed_per_pnode[pnode] = fixed

        layers = []
        constraints = []
        previous_relaxations = set()
        pending_relaxations = []
        for pnode in self.select_actions:
            #print pnode
            log.debug("cond: %s", str(map(str,pnode.original_preconds)))
            det_constraints = []
            prob_constraints = []
            constants = set()
            indep_vars = set()
            depvars = set()
            deps = simplegraph.Graph()
            for svar, val in pnode.original_preconds:
                if val.is_instance_of(pddl.t_number):
                    continue
                constants |= set(svar.args + svar.modal_args + (val,))
                if svar.modality in (mapl.hyp, mapl.indomain):
                    if svar.modality == mapl.hyp:
                        indep_vars |= set(svar.args + svar.modal_args)
                    constr = pddl.state.Fact(svar.nonmodal(), svar.modal_args[0])
                    prob_constraints.append(constr)
                    deps[svar.modal_args[0]] |= set(svar.args)
                else:
                    constr = pddl.state.Fact(svar, val)
                    det_constraints.append(constr)
                    deps[val] |= set(svar.args)

            constants -= set(pnode.full_args)
            depvars |= deps.undirected().succ(indep_vars) - indep_vars
            depvars &= set(pnode.full_args)
            indep_vars |= set(pnode.full_args) - depvars
            indep_vars &= set(pnode.full_args)
            relaxation_order = deps.topological_sort()
            dep_relaxations = [v for v in reversed(relaxation_order) if v in depvars]
            indep_relaxations = [v for v in relaxation_order if v in indep_vars]
            #print "independent vars:", map(str, indep_relaxations)
            #print "dependent vars:", map(str, dep_relaxations)

            for c in chain(det_constraints):
                if c not in constraints:
                    constraints.append(c)
                    
            fixed = fixed_per_pnode[pnode]
            for v in chain([None], dep_relaxations):
                if v is not None:
                    if deps[v] and not (deps[v] - indep_vars):
                        #print "Fake relaxation:", v
                        continue
                    #print "removing", v
                    fixed.discard(v)
                    previous_relaxations.add(v)
                    
                cnew = []
                for c in constraints:
                    if c.value not in fixed | constants or all(v in fixed|constants for v in chain(c.svar.args, [c.value])):
                        continue
                    cnew.append(c)
                
                log.debug("fixed on this layer: %s", str(map(str, fixed)))
                c2 = transform_constraints(fixed | constants, cnew)
                fc = partial_problem.ObjectsConstraint(fixed)
                layers.append([fc]+c2)

            pending_relaxations = [v for v in pending_relaxations if v not in dep_relaxations + indep_relaxations]
            pending_relaxations += indep_relaxations
            #print "pending:", map(str, pending_relaxations)

        for v in pending_relaxations:
            if v is not None:
                #print "removing", v
                fixed.discard(v)

            cnew = []
            for c in constraints:
                if c.value not in fixed | constants or all(v in fixed| constants for v in chain(c.svar.args, [c.value])):
                    continue
                cnew.append(c)

            log.debug("fixed on this layer: %s", str(map(str, fixed)))
            c2 = transform_constraints(fixed | constants, cnew)
            fc = partial_problem.ObjectsConstraint(fixed)
            layers.append([fc]+c2)
             
        for i, constraints in enumerate(layers):
            log.debug("Layer %d", i)
            log.debug("Fixed: %s", str(constraints[0]))
            log.debug("Constraints: %s", str(map(str, constraints[1:])))
            log.debug("")
        return layers


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
            log.debug("cond: %s", str(map(str,pnode.original_preconds)))
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
            for svar, val in pnode.original_preconds:
                if svar.modality in (mapl.commit, mapl.indomain) :
                    new_var_relaxations |= set(svar.args)
                    new_value_relaxations |= set(svar.modal_args)
                #elif supporter[svar] != self.plan.init_node:
                #    new_var_relaxations |= set(svar.args)
                #    new_value_relaxations.add(val)
                elif svar.function.type != pddl.t_number:
                    new_var_relaxations |= set(svar.args)
                    new_value_relaxations.add(val)
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
            
    def create_goal_actions(self, goals, fail_counts, domain):
        commit_actions = []

        confirm_score = global_vars.config.dt.confirm_score
        failure_multiplier = global_vars.config.dt.failure_multiplier

        a = pddl.Action("cancel", [], None, None, domain)
        b = pddl.builder.Builder(a)
        a.precondition = b.cond('not', ('done', ))
        a.effect = b.effect('and', ('done', ), ('assign', ('reward',), 0))
        commit_actions.append(a) # comment out to disable the cancel action
        
        for fact in goals:
            if fact.svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                svar = fact.svar.nonmodal()
            else:
                log.debug("Goal not yet supported: %s", str(fact))
                continue

            mult = failure_multiplier**fail_counts[fact]

            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args())
            domain.add(svar.get_args())
            
            val = pddl.Parameter("?val", svar.function.type)
            name = "commit-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            a = pddl.Action(name, [val], None, None, domain)
            b = pddl.builder.Builder(a)
            
            #a.precondition = b.cond('and', ('not', (mapl.committed, [term])))
            a.precondition = b.cond('not', ('done', ))
            commit_effect = b.effect(mapl.committed, [term])
            reward_effect = b('when', ('=', term, val), ('assign', ('reward',), confirm_score * mult ))
            penalty_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), -confirm_score * mult))
            done_effect = b.effect('done')
            a.effect = b.effect('and', reward_effect, penalty_effect, commit_effect, done_effect)
            
            commit_actions.append(a)

        max_level = 0
        disconfirm = []
        for pnode, level in self.select_actions:
            for svar, val in pnode.effects:
                if svar.modality == mapl.commit:
                    max_level = max(max_level, level)
                    nmvar = svar.nonmodal()
                    #if nmvar not in goals:
                    disconfirm.append((nmvar, svar.modal_args[0], level))
                    break # only one disconfirm per node

        if not disconfirm:
            return commit_actions
                        
        dis_score = float(confirm_score)/(2**(len(disconfirm)-1))
        disconfirm_actions = []
        for svar, val, level in disconfirm:
            dis_reward = float(confirm_score)/(2**(max_level-level))
            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args() + [val])
            domain.add(svar.get_args() + [val])
            
            name = "disconfirm-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            a = pddl.Action(name, [], None, None, domain)
            b = pddl.builder.Builder(a)

            #conds = [b.cond('not', (dtpddl.committed, term))]
            #conds = [b.cond('not', ('done',))]
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
            reward_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), dis_reward))
            penalty_effect = b('when', ('=', term, val), ('assign', ('reward',), -dis_reward))
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

    def compute_subproblems(self, prob_state):
        # import debug
        # debug.set_trace()
        
        problems =  []
        all_objects = set(prob_state.problem.get_all_objects(pddl.t_object))
        #all_objects = cast_state.objects | cast_state.generated_objects | self.domain.constants
        problems.append(partial_problem.PartialProblem(prob_state, all_objects,  [], [], self.dt_rules, self.domain))
        i = len(self.relaxation_layers)-1
        prev_constraints = []
        for constraints in reversed(self.relaxation_layers):
            # Iterate over relaxation layers first (from most to least relaxed)
            log.debug("Next layer %d:", i)
            problems.append(partial_problem.PartialProblem(prob_state, problems[-1].objects, prev_constraints, constraints, self.dt_rules, self.domain))
                    
            i -= 1
            prev_constraints = constraints

        return problems

    def create_state_trees(self, prob_state, objects=None):
        if not objects:
            objects = prob_state.problems.objects | prob_state.problems.domain.constants
        return StateTreeNode.create_root(prob_state, objects, self.dt_rules)

    def create_problem(self, prob_state, domain):
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

        problem = self.subproblems[selected]
        objects = problem.objects

        self.selected_subproblem = selected
        rulefuncs = set(r.function for r in self.dt_rules if not r.conditions) # roots of the prob.tree

        trees = self.create_state_trees(prob_state, objects) 
        hstate = HierarchicalState([], prob_state.problem)
        for t in trees:
            if t.svar.function in rulefuncs or len(t) == 1: # HACK! evil HACK!
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

    def reduce_state(self, limit):
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
            d = dict((v,set(s)) for v,s in d.iteritems())
            for svar, val in it:
                s = d.setdefault(svar, set())
                s.add(val)
            return d

        choices = self.extract_choices()
        log.debug("initial choices: %s", map(str, choices))
        o_funcs = self.observable_functions()

        for n in self.pnodes:
            get_level(n, 0)
            
        nodes_by_level = defaultdict(set)
        for n, l in levels.iteritems():
            nodes_by_level[l].add(n)
        node_order = sorted(nodes_by_level.iteritems(), key = lambda (l,n): -l)

        init = update({}, [(svar, val) for svar, val in choices.iteritems() if svar.function != dtpddl.selected])
        H_init = self.entropy(init, {}, limit)
        log.debug("initial entropy: %.4f", H_init)
        # print "initial entropy: %.4f" % H_init
        # for svar, val in choices.iteritems():
        #     others = dict((s, set([v])) for s, v in choices.iteritems() if s.function != dtpddl.selected and s != svar)
        #     print "H(init|%s): %.4f" %(str(svar), self.entropy(others, update({}, [(svar,val)]), limit))
            
            
        selected = update({}, choices.iteritems())
        node_queue = []
        # print "initial choices:", [str(state.Fact(s,v)) for s,v in choices.iteritems()]
        while node_order:
            #while total_size(nodes, selected) < limit:
            level, this_nodes = node_order.pop(0)
            all_facts = set()
            for n in this_nodes:
                for facts in n.add_facts(choices):
                    all_facts |= facts
            all_facts = set(f for f in all_facts if f.svar.function in o_funcs)
            # node_queue = chain(*itertools.izip_longest(*[n.add_facts(choices) for n in this_nodes]))
            # all_new_facts = set(chain(*[f for f in node_queue]))
            # for fact in sorted(all_facts, key=lambda f: self.cond_entropy(init, update({}, [f]), limit)):
            all_facts_with_entropy = [(f, self.entropy(init, update({}, [f]), limit)) for f in all_facts]
            for fact, H in sorted(all_facts_with_entropy, key=lambda (f,H): H):
                next = update(selected, [fact])
                tsize = len(self.compute_states(next, limit))
                if tsize <= limit and H <= H_init - 0.05:
                    log.debug("added: %s (%d, %.3f)", str(fact), tsize, H)
                    selected = next
                else:
                    log.debug("skipped: %s (%d, %.3f)", str(fact), tsize, H)
                    pass

            
            
            # for added_facts in node_queue:
            #     for fact in added_facts:
            #         next = update(selected, [fact])
            #         tsize = len(self.compute_states(next, limit))
            #         H = self.cond_entropy(update({}, choices.iteritems()), update({}, [fact]), limit)
            #         #tsize = total_size(nodes, next)
            #         if tsize < limit:
            #             print "added: %s (%d)" % (str(fact), tsize)
            #             selected = next
            #         else:
            #             print "skipped: %s (> %d)" % (str(fact), tsize)
            #             pass
        return selected

    def compute_states(self, selected_facts, limit, order=None):
        if not order:
            order = list(selected_facts.iterkeys())
        svar_order = dict((var, i) for i, var in enumerate(order))
        undef = (pddl.UNKNOWN,)*len(selected_facts)
        
        def multiply_states(sset1, sset2):
            result = defaultdict(lambda: 0.0)
            # print "--------------------------"
            # for s, p in sset1.iteritems():
            #     print map(str,s), p
            # print
            # for s, p in sset2.iteritems():
            #     print map(str,s), p
            
            # print len(sset1), len(sset2)
            for (s1, p1), (s2, p2) in product(sset1.items(), sset2.items()):
                res = []
                for v1, v2 in itertools.izip(s1, s2):
                    if v1 == pddl.UNKNOWN:
                        res.append(v2)
                    else:
                        if v2 == pddl.UNKNOWN:
                            res.append(v1)
                        else:
                            res = None
                            break
                if res:
                    result[tuple(res)] += p1*p2
            # print len(result)
            return result
                
        def build_state(node):
            result = defaultdict(lambda: 0.0)
            p_total = 0.0
            for val, (p, nodes, facts) in node.children.iteritems():
                p_total += p
                sfacts = [pddl.UNKNOWN]*len(selected_facts)
                for svar, v in chain(facts.iteritems(), [(node.svar, val)]):
                    if svar in selected_facts and v in selected_facts[svar]:
                        # print "set:", svar, v
                        sfacts[svar_order[svar]] = v
                branch_states = {tuple(sfacts) : 1.0}
                # print map(str, sfacts)
                for n in nodes:
                    branch_states = multiply_states(branch_states, build_state(n))
                    if len(branch_states) > limit:
                        return branch_states
                    
                for bs, bp in branch_states.iteritems():
                    # print map(str, bs) , p*bp
                    result[bs] += p*bp

                if len(result) > limit:
                    return result
            if p_total < 1.0:
                result[undef] += 1.0 - p_total
                
            return result

        states = {undef : 1.0}
        for n in self.pnodes:
            states = multiply_states(states, build_state(n))
            if len(states) > limit:
                break
        return states

    def entropy(self, facts, condfacts, limit):
        # print "\ngiven:"
        # for svar, vals in facts.iteritems():
        #     print svar, map(str, vals)
        # print " cond:"
        # for svar, vals in condfacts.iteritems():
        #     print svar, map(str, vals)

        # for svar, vals in facts.iteritems():
        #     if svar in condfacts:
        #         condfacts[svar] -= vals
        # for svar in [svar for svar, vals in condfacts.iteritems() if not vals]:
        #     del condfacts[svar]
                
        # if any(svar in condfacts for svar in facts.iterkeys()) or not condfacts:
        #     return 0
        
        order = list(svar for svar in facts.iterkeys() if svar not in condfacts) + list(condfacts.iterkeys())
        svar_order = dict((var, i) for i, var in enumerate(order))
        
        relevant_facts = {}
        for svar, vals in facts.iteritems():
            relevant_facts[svar] = set(vals)
        for svar, vals in condfacts.iteritems():
            vset = relevant_facts.setdefault(svar, set())
            vset |= vals

        states = self.compute_states(relevant_facts, limit, order)
        cfdict = defaultdict(lambda: 0.0)
        if condfacts:
            for s, p in states.iteritems():
                cf = s[-len(condfacts):]
                cfdict[cf] += p
        H = 0
        for s, p in states.iteritems():
            cf = s[-len(condfacts):]
            # print map(str, s), "  ", map(str, cf)
            # print "+ %.5f * log(%.5f/%.5f) = %.5f" % (p, cfdict[cf], p, p * math.log(cfdict[cf]/p,2) )
            if p <= 0.000000001:
                continue
            if condfacts:
                H += p * math.log(cfdict[cf]/p,2)
            else:
                H -= p * math.log(p,2)
        # print "H:", H
        return H
        
    

    def create_prob_state(self, rules, state):
        pass

    def get_observations_for(self, action):
        for o in self.domain.observe:
            if not o.execution:
                yield o
            for ex in o.execution:
                if not ex.negated and ex.action == action:
                    yield o

    def relevant_actions(self, goals):
        @pddl.visitors.collect
        def collect_vars(elem, results):
            if isinstance(elem, pddl.Literal):
                return state.StateVariable.from_literal(elem)
            
        def get_abstract_observations(observe, agent):
            @pddl.visitors.collect
            def eff_collector(eff, results):
                if isinstance(eff, pddl.ConditionalEffect):
                    return eff.condition.visit(collect_vars)
                
            observe.instantiate(observe.args)
            for svar in chain(pddl.visitors.visit(observe.precondition, collect_vars, []), observe.effect.visit(eff_collector)):
                if svar.function in self.prob_functions:
                    yield svar.as_modality(mapl.knowledge, [agent])
            observe.uninstantiate()
                
        def get_abstract_transitions(action):
            action.instantiate(action.args) # workaround because StateVariables cannot be created from ungrounded literals
            pre = set(pddl.visitors.visit(action.precondition, collect_vars, set()))
            eff = set(pddl.visitors.visit(action.effect, collect_vars, set()))
            for o in self.get_observations_for(action):
                eff |= set(get_abstract_observations(o, action.agents[0]))
            action.uninstantiate()
            return pre , eff

        relevant_actions = defaultdict(lambda: defaultdict(set))
        for a in self.domain.actions:
            pre, eff = get_abstract_transitions(a)
            for p, e in product(pre, eff):
                relevant_actions[e.function][p.function].add(a)
                
        open = set(svar.function for svar, val in goals)

        closed = set()
        result = set()
        while open:
            var = open.pop()
            closed.add(var)
            for pre, actions in relevant_actions[var].iteritems():
                result |= actions
                if pre not in closed:
                    open.add(pre)

        for a in result:
            print a.name
                    
        return result

    
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

    def observable_functions(self):
        @pddl.visitors.collect
        def obs_visitor(eff, results):
            if isinstance(eff, pddl.ConditionalEffect):
                return eff.condition.visit(pddl.visitors.collect_non_builtins)

        obs_funcs = set()
        for o in self.domain.observe:
            obs_funcs |= set(pddl.visitors.visit(o.precondition, pddl.visitors.collect_non_builtins, set()))
            obs_funcs |= set(pddl.visitors.visit(o.effect, obs_visitor , set()))
        return obs_funcs
    
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
    def __init__(self, svar):
        self.svar = svar

    def consolidate(self, existing=None):
        values = set()
        if existing:
            values |= existing.keys()
            tree = existing
        else:
            tree = StateTreeNode(self.svar)
            
        for val, (subtrees, p, marginal) in self.iteritems():
            assert val not in values
            values.add(val)
            st_by_var = {}
            for sub in subtrees:
                if sub.svar in st_by_var:
                    sub.consolidate(existing = st_by_var[sub.svar])
                else:
                    st_by_var[sub.svar] = sub.consolidate()
            tree[val] = (st_by_var.values(), p, marginal)
        return tree

    def add_state(self, st, ratio):
        for val, (subtrees, p, marginal) in self.iteritems():
            if marginal:
                val = pddl.UNKNOWN

            if p == 1.0 and not marginal:
                sub = st
            else:
                if st.has_substate(self.svar, val):
                    oldp, sub = st.get_substate(self.svar, val)
                    st.add_substate(self.svar, val, sub, (p*ratio)+(oldp* (1-ratio)))
                else:
                    sub = HierarchicalState([], parent=st)
                    st.add_substate(self.svar, val, sub, (p*ratio))
                    for t in subtrees:
                        t.create_state(sub)
            sub[self.svar] = val

    def create_state(self, st):
        for val, (subtrees, p, marginal) in self.iteritems():
            if marginal:
                val = pddl.UNKNOWN

            if p == 1.0 and not marginal:
                sub = st
            elif p <= 0.001:
                continue
            else:
                if st.has_substate(self.svar, val):
                    oldp, sub = st.get_substate(self.svar, val)
                    st.add_substate(self.svar, val, sub, p+oldp)
                    ratio = p/(p + oldp)
                    for t in subtrees:
                        t.add_state(sub, ratio)
                else:
                    sub = HierarchicalState([], parent=st)
                    st.add_substate(self.svar, val, sub, p)
                    for t in subtrees:
                        t.create_state(sub)
            sub[self.svar] = val

    @staticmethod
    def create(rule, st, objects, all_rules, cache):
        svar = state.StateVariable(rule.function, state.instantiate_args(rule.args))
        tree = StateTreeNode(svar)
        
        log.debug("creating subtree for %s", str(svar))
        for p, value in rule.values:
            pval = st.evaluate_term(p) # evaluate in original state because probs for marginals could be gone in this partial state
            if pval == pddl.UNKNOWN or pval.value < 0.01:
                continue
            val = st.evaluate_term(value)
            #t0 = time.time()
            tree.create_subtree(pval.value, val, rule, st, objects, all_rules, cache, marginal=(val not in objects))
            #print "tree for %s=%s took %.2f secs" % (str(self.svar), val.name, time.time()-t0)
        return tree

    def create_subtree(self, prob, value, rule, st, objects, all_rules, cache, marginal=False):
        log.debug("creating subtrees for %s=%s (marginal=%s)", str(self.svar), value, str(marginal))
        fact = state.Fact(self.svar, value)
        if fact in cache:
            #print "cache hit"
            self[value] = (cache[fact], prob, marginal)
            return

        if rule is None:
            self[value] = ([], prob, marginal)
            return

        rules = [r for r in all_rules if r.depends_on(rule)]
        log.debug("rules depending on this: %s", ", ".join(r.name for r in rules))
        
        subtrees = []
        for r in rules:
            matched_cond = None
            matched_args = {}
            for lit in r.conditions:
                if lit.predicate == pddl.equals and lit.args[0].function == rule.function:
                    matched_cond = lit
                    for a, a2 in zip(lit.args[0].args + [lit.args[1]], self.svar.args + (value,)):
                        matched_args[a.object] = a2
                    break
                elif lit.predicate == rule.function:
                    matched_cond = lit
                    v = pddl.TRUE if not lit.negated else pddl.FALSE
                    for a, a2 in zip(lit.args + [v], self.svar.args + (value,)):
                        matched_args[a.object] = a2
                    break
            if matched_cond and any(a.__class__ == a2.__class__ == pddl.TypedObject and a != a2 for a,a2 in matched_args.iteritems()):
                continue

            def get_objects(arg):
                if arg in matched_args:
                    return [matched_args[arg]]
                if arg in r.get_value_args():
                    return list(st.problem.get_all_objects(arg.type))
                return [o for o in objects if o.is_instance_of(arg.type)]


            args = r.args+r.add_args
            for mapping in r.smart_instantiate(r.get_inst_func(st, matched_cond), args, [get_objects(a) for a in args], st.problem):
                subtrees.append(StateTreeNode.create(r, st, objects, all_rules, cache))
        
        cache[fact] = subtrees
        self[value] = (subtrees, prob, marginal)

    @staticmethod
    def create_from_fact(fact, st, objects, all_rules, cache):
        tree = StateTreeNode(fact.svar) 
        rules_for_fact = [r for r in all_rules if r.function == fact.svar.function]
        rule = (rules_for_fact[0] if rules_for_fact else None)

        if isinstance(fact.value, pddl.TypedObject):
            #t0 = time.time()
            tree.create_subtree(1.0, fact.value, marginal=(fact.value not in objects))
            #print "tree for %s=%s took %.2f secs" % (str(self.svar), self.value.name, time.time()-t0)
        else:
            for val, p in fact.value.iteritems():
                #t0 = time.time()
                tree.create_subtree(p, val, rule, st, objects, all_rules, cache, marginal=(val not in objects))
                #print "tree for %s=%s took %.2f secs" % (str(self.svar), val.name, time.time()-t0)
        return tree

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
        
        detstate = st.determinized_state(0.05, 0.95)
        for fact in st.iterdists():
            if fact.value.value == pddl.UNKNOWN or any(a not in objects for a in fact.svar.args):
                continue
            sub = StateTreeNode.create_from_fact(fact, detstate, objects, rules, cache)
            subtrees.append(sub)
            done.add(sub.svar)
                
        for r in nodep_rules:
            def get_objects(arg):
                if arg in r.get_value_args():
                    return list(detstate.problem.get_all_objects(arg.type))
                return [o for o in objects if o.is_instance_of(arg.type)]
            
            args = r.args+r.add_args
            for mapping in r.smart_instantiate(r.get_inst_func(detstate), args, [get_objects(a) for a in args], detstate.problem):
                svar = state.StateVariable(r.function, state.instantiate_args(r.args))
                if svar in done:
                    continue
                subtrees.append(StateTreeNode.create(r, detstate, objects, rules, cache))

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
            value = pddl.types.TypedNumber(value)
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
