import time, math, itertools
from itertools import chain, product

from collections import defaultdict
from standalone import task, config, pddl, plans, relaxed_exploration
from standalone.pddl import state, dtpddl, mapl, translators, visitors, effects

import standalone.globals as global_vars
import simplegraph
import partial_problem

log = config.logger("dt")


class DTProblem(object):
    def __init__(self, plan, pnodes, failed_goals, prob_functions, global_rel_facts, domain):
        self.plan = plan
        self.pnodes = pnodes
        self.domain = domain
        self.failed_goals = failed_goals
        self.prob_functions = prob_functions
        self.subplan_actions = []
        self.dt_plan = []
        
        self.goals, self.assumptions = self.create_goals(plan)
        if not self.goals:
            return

        try:
            self.dt_rules = domain.dt_rules
        except:
            self.dt_rules = []
        self.prob_functions |= set(r.function for r in self.dt_rules)
        self.global_relevant = global_rel_facts
            
        self.dtdomain = self.create_dt_domain(domain)

        # dom_str, prob_str = DTPDDLOutput().write(self.problem)
        # print "\n".join(dom_str)
        # print "\n".join(prob_str)
        
    def initialize(self, prob_state):
        self.state = prob_state
        self.detstate = prob_state.determinized_state(0.05, 0.95)
        
        def node_filter(node):
            for f in node.all_facts():
                if f.svar.function == dtpddl.selected:
                    continue
                if f.svar not in self.state or not self.state.is_det(f.svar):
                    return True
            return False

        self.pnodes = filter(node_filter, self.pnodes)
            
        facts = self.reduce_state(global_vars.config.dt.max_state_size)
        self.selected_facts = set()
        add_objects = set()
        for var, vals in facts.iteritems():
            add_objects.update(var.args)
            add_objects.update(var.modal_args)
            for v in vals:
                add_objects.add(v)
                self.selected_facts.add(state.Fact(var, v))
                log.debug("selected: %s, %s", var, v)

        # for o, mapping in self.get_observations(self.selected_facts):
        #     print "(%s %s)" % (o.name, " ".join(mapping.get(a,a).name for a in o.args))

        used_objects = set()

        o_actions = list(self.get_observe_actions(self.selected_facts))
        # for a, mapping in o_actions:
        #     print "(%s %s)" % (a.name, " ".join(mapping.get(a,a).name for a in a.args))

        t0 = time.time()
        actions, explored_facts = relaxed_exploration.explore(self.domain.actions, set(), self.detstate, self.domain, o_actions)
        print "total time for exploration: %.2f" % (time.time()-t0)

        for a, args in actions:
            used_objects |= set(args)
                
        def objects(svar, val):
            return set(svar.args + svar.modal_args + (val,))
                
        pruned_objects = set()
        used_objects |= set(chain(*[pnode.full_args for pnode, level in self.assumptions]))
        log.debug("used_objects objects: %s", map(str, used_objects))

        new_objects = (self.state.problem.objects | add_objects) - pruned_objects

        used_objects |= self.domain.constants

        opt = "maximize"
        opt_func = pddl.FunctionTerm(pddl.dtpddl.reward, [])
        init = [f.to_init() for f in self.state.deterministic() if not f.value.is_instance_of(pddl.t_number) and (objects(*f) < used_objects)]
        # for f in self.state.deterministic():
        #     print f.to_init(),  (objects(*f) < used_objects)
        
        for n in self.pnodes:
            peff = n.to_init(self.selected_facts)
            used_objects |= set(visitors.visit(peff, pddl.visitors.collect_constants, [])) - self.domain.constants
            if peff:
                init.append(peff)
                
        self.problem = pddl.Problem("cogxtask", used_objects - self.domain.constants, init, None, self.dtdomain, opt, opt_func)
        self.problem.goal = pddl.Conjunction([])

        self.abstract_state = pddl.prob_state.ProbabilisticState.from_problem(self.problem)

        self.goal_actions = self.create_goal_actions(self.goals, self.failed_goals, self.selected_facts, explored_facts, self.dtdomain)
        self.dtdomain.actions += [a for a in self.goal_actions]
        self.dtdomain.name2action = None
        
        #self.selected_subproblem = -1

        #for pnode in self.subplan_actions:
        #    pnode.status = plans.ActionStatusEnum.IN_PROGRESS
        
        #self.relaxation_layers = self.compute_restrictions_new()
        #self.relaxation_layers = self.compute_restrictions()
        #self.subproblems = self.compute_subproblems(self.state)
        #self.problem = self.create_problem(self.state, self.dtdomain)

    def explore_state(self, goal_actions):
        pass

    def extract_choices(self):
        choices = {}
        for pnode, level in self.assumptions:
            # print pnode, level, map(str, pnode.effects)
            for fact in pnode.effects:
                if fact.svar.modality == mapl.commit:
                    fact = state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])
                if not fact.svar.modality and fact.svar.get_type().equal_or_subtype_of(pddl.t_object):
                    choices[fact.svar] = (fact.value, level)
        return choices

    def write_dt_input(self, domain_fn, problem_fn, dt_id=0):
        self.problem.name = "cogx-dt-task-%d" % dt_id
        self.problem.domain.name = "cogx-dt-domain-%d" % dt_id
        DTPDDLOutput().write(self.problem, domain_fn=domain_fn, problem_fn=problem_fn)


    def get_observe_actions(self, relevant_facts):
        for o, mapping in self.get_observations(relevant_facts):
            for e in o.execution:
                action = e.action
                o_a_mapping = dict(zip(e.args, action.args))
                a_mapping = {}
                for oarg, val in mapping.iteritems():
                    if oarg in o_a_mapping:
                        a_mapping[o_a_mapping[oarg]] = val
                yield action, a_mapping

    def get_observations(self, relevant_facts):
        relevant_functions = set(f.svar.function for f in relevant_facts)
        @visitors.collect
        def atom_visitor(elem, results):
            if isinstance(elem, pddl.LiteralCondition):
                if elem.predicate != dtpddl.observed and pddl.translators.get_function(elem) in relevant_functions:
                    return [elem]

        @visitors.collect
        def effect_visitor(elem, results):
            if isinstance(elem, pddl.ConditionalEffect):
                return elem.condition.visit(atom_visitor)
                
        for o in self.domain.observe:
            sensable_atoms = []
            if o.precondition:
                sensable_atoms += o.precondition.visit(atom_visitor)
            sensable_atoms += o.effect.visit(effect_visitor)
            for lit in sensable_atoms:
                for fact in relevant_facts:
                    mapping = fact.match_literal(lit)
                    if mapping:
                        yield o, mapping
            
        
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

        goal_facts = set()
        assumptions = []
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            if pnode.action.name in observe_actions:
                #TODO: only add an action if the observe effect supports a later action
                for fact in pnode.effects:
                    if fact.svar.function not in (pddl.builtin.total_cost, ):
                        goal_facts.add(fact)
                self.subplan_actions.append(pnode)
                assumptions += [(pnode,0)] + find_restrictions(pnode,0)
                break
            
            if pnode.action.name not in observe_actions and self.subplan_actions:
                break
        
        self.remaining_costs = 0
        for pnode in plan.topological_sort():
            if pnode.status != plans.ActionStatusEnum.EXECUTED and not pnode.is_virtual() and pnode not in self.subplan_actions:
                self.remaining_costs += pnode.cost
            
        return goal_facts, assumptions

    def expected_observation(self, svar):
        #This is a hack to prevent pcogx from segfaulting
        if svar.modality != dtpddl.observed:
            return True # Don't handle those
        ground_fact = pddl.state.Fact(svar.nonmodal(), svar.modal_args[0])
        if ground_fact not in self.selected_facts:
            print ground_fact
            print map(str, self.selected_facts)
            return False
        return True


    def replanning_neccessary(self, new_state):
        # if self.selected_subproblem == -1:
        #    return True
        
        # new_objects = new_state.problem.objects - self.state.problem.objects
        # problem = self.subproblems[self.selected_subproblem]
        
        # for o in new_objects:
        #    if all(c.matches(o, self.state) for c in problem.constraints):
        #        return True
           
        return False

    def calc_disconfirm_gain(self, dis_fact, dep_var):
        def node_filter(node, val, p, facts):
            for svar, v in chain([(node.svar, val)], facts.iteritems()):
                if svar == dis_fact.svar and v == dis_fact.value:
                    return False
            return True

        if self.state.is_det(dep_var):
            return 0

        relevant_facts = {dep_var : self.global_relevant[dep_var]}
        H = -sum(p*math.log(p,2) for p in self.state[dep_var].itervalues() if p > 0)
        cH = self.entropy(relevant_facts, {}, 100, node_filter)
        print "H(%s) - H(.|!%s) = %.2f - %.2f =  %.2f" % (str(dep_var), str(dis_fact), H, cH, H - cH)
        return max(H - cH, 0)
            
    def create_goal_actions(self, goals, fail_counts, selected, changed_facts, domain):
        commit_actions = []
        dt_conf = global_vars.config.dt

        def clear_state_effect():
            effs = [f.to_effect() for f in chain(selected, changed_facts) ]
            return pddl.ConjunctiveEffect([])

        for f in chain(selected, changed_facts):
            for o in f.svar.get_args():
                domain.add_constant(o)
            domain.add_constant(f.value)

        # confirm_score = global_vars.config.dt.confirm_score
        # failure_multiplier = global_vars.config.dt.failure_multiplier

        a = pddl.Action("cancel", [], None, None, domain)
        b = pddl.builder.Builder(a)
        a.precondition = b.cond('not', ('done', ))
        a.effect = b.effect('and', ('done', ), ('assign', ('reward',), 0), clear_state_effect())
        commit_actions.append(a) # comment out to disable the cancel action

        self.confirm_dict = {}
        self.disconfirm_dict = {}
        
        for fact in goals:
            if fact.svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                svar = fact.svar.nonmodal()
            else:
                log.debug("Goal not yet supported: %s", str(fact))
                continue

            assert svar in self.global_relevant
            mult = dt_conf.failure_multiplier**fail_counts[fact]

            term = pddl.Term(svar.function, svar.get_args())
            for o in svar.get_args():
                domain.add_constant(o)

            rel_facts = [f for f in selected if f.svar == svar]
            for f in rel_facts:
                if f.value not in self.global_relevant[f.svar]:
                    continue
                if isinstance(self.abstract_state[svar], pddl.TypedObject):
                    p = 1.0 if self.abstract_state[svar] == f.value else 0.0
                p = self.abstract_state[svar][f.value]
                if p == 0.0:
                    continue

                if dt_conf.commitment_mode in ("conf", "all"):
                    reward = dt_conf.total_reward - self.remaining_costs
                    penalty = -dt_conf.total_reward - self.remaining_costs
                else:
                    reward = dt_conf.confirm_score * mult
                    penalty = -reward * (p)/(1-p) if p != 1.0 else 0
                                    
                #val = pddl.Parameter("?val", svar.function.type)
                name = "commit-%s-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()), f.value.name)
                self.confirm_dict[name] = f
                a = pddl.Action(name, [], None, None, domain)
                b = pddl.builder.Builder(a)

                #a.precondition = b.cond('and', ('not', (mapl.committed, [term])))
                a.precondition = b.cond('not', ('done', ))
                commit_effect = b.effect(mapl.committed, [term])
                reward_effect = b('when', ('=', term, f.value), ('assign', ('reward',), reward ))
                penalty_effect = b('when', ('not', ('=', term, f.value)), ('assign', ('reward',), penalty))
                done_effect = b.effect('done')
                a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, clear_state_effect())
            
                commit_actions.append(a)

        max_level = 0
        disconfirm = []
        for pnode, level in self.assumptions:
            for svar, val in pnode.effects:
                if svar.modality == mapl.commit:
                    max_level = max(max_level, level)
                    nmvar = svar.nonmodal()
                    #if nmvar not in goals:
                    disconfirm.append((nmvar, svar.modal_args[0], level))
                    break # only one disconfirm per node

        if not disconfirm:
            return commit_actions

        disconfirm = set((svar, val) for svar, val, _ in disconfirm)
        
        # dis_score = float(confirm_score)/(2**(len(disconfirm)-1))
        disconfirm_actions = []
        for svar, val in disconfirm:
            dis_fact = state.Fact(svar, val)
            dH = 0
            count = 0
            for goalvar in set(g.svar.nonmodal() for g in goals):
                if goalvar in self.global_relevant:
                    dH +=  self.calc_disconfirm_gain(dis_fact, goalvar)
                    count += 1

            if dH == 0:
                continue
            dH = dH/count
                
            dis_reward = float(dt_conf.confirm_score) * dH #/(2**(max_level-level))
            if isinstance(self.abstract_state[svar], pddl.TypedObject):
                continue # no use in disconfirming known facts
            p = self.abstract_state[svar][val]
            dis_penalty = -dis_reward * (1-p)/p

            
            term = pddl.Term(svar.function, svar.get_args())
            # p = self.abstract_state[svar][val]
            # if p < 0.001 or p > 0.999:
            #     continue # no use in disconfirming known facts

            # dis_penalty = -dis_score * (1-p)/p
            
            
            name = "disconfirm-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            self.disconfirm_dict[name] = state.Fact(svar, val)
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
            penalty_effect = b('when', ('=', term, val), ('assign', ('reward',), dis_penalty))
            done_effect = b.effect('done')
            a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, clear_state_effect())
            
            disconfirm_actions.append(a)

        return commit_actions + disconfirm_actions

    def get_dt_results(self, action):
        if action.name in self.confirm_dict:
            return self.confirm_dict[action.name], None
        if action.name in self.disconfirm_dict:
            return None, self.disconfirm_dict[action.name]
        return None, None

    def create_dt_domain(self, dom):
        dtdomain = dom.copy()
        dtdomain.name = "dt-%s" % dom.name
        dtdomain.annotations =  pddl.translators.Translator.get_annotations(dom)
        return dtdomain

    def recompute_problem(self, new_state):
        self.state = new_state
        self.subproblems = self.compute_subproblems(self.state)
        self.problem, hstate = self.create_problem(self.state, self.dtdomain)
        
        self.goal_actions = self.create_goal_actions(self.goals, hstate, self.dtdomain)
        self.dtdomain.actions += [a for a in self.goal_actions]
        self.dtdomain.name2action = None
        
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
        return problem, hstate

    def reduce_state(self, limit):
        levels = defaultdict(lambda: -1)
        def get_level(node, level):
            cval, clevel = choices.get(node.svar, (None, -1))
            log.debug("get level: %s, %s", str(node), str(cval))
            if cval and levels[node] < level and cval in node.children:
                levels[node] = level
                p, nodes, facts = node.children[cval]
                for svar, val in facts.iteritems():
                    choices[svar] = (val, clevel)
                    # cfacts.update(facts)
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

        cfacts = dict((svar, val) for svar, (val, level) in choices.iteritems())
        init = update({}, [(svar, val) for svar, val in cfacts.iteritems() if svar.function != dtpddl.selected])
        H_init = self.entropy(init, {}, limit)
        log.debug("initial entropy: %.4f", H_init)
        # print "initial entropy: %.4f" % H_init
        
        choice_by_level = defaultdict(set)
        for svar, (val, level) in choices.iteritems():
            choice_by_level[level].add((svar, val))

        toplevel = update({}, choice_by_level[1])
            
        selected = {}
        for level, facts in sorted(choice_by_level.iteritems(),key=lambda (l,f): l):
            if selected:
                for svar, val in facts:
                    log.debug("H(top|%s): %.4f", str(svar), self.entropy(toplevel, update({}, [(svar,val)]), limit))
                    log.debug("H(parents|%s): %.4f", str(svar), self.entropy(selected, update({}, [(svar,val)]), limit))
            selected = update(selected, facts)
                

        for svar, val in cfacts.iteritems():
            others = dict((s, set([v])) for s, v in cfacts.iteritems() if s.function != dtpddl.selected and s != svar)
            log.debug("H(init|%s): %.4f", str(svar), self.entropy(others, update({}, [(svar,val)]), limit))
            
        # selected = update({}, cfacts)
        node_queue = []
        # print "initial choices:", [str(state.Fact(s,v)) for s,v in choices.iteritems()]
        while node_order:
            #while total_size(nodes, selected) < limit:
            level, this_nodes = node_order.pop(0)
            # print level, map(str, this_nodes)
            all_facts = set()
            for n in this_nodes:
                for facts in n.add_facts(cfacts):
                    all_facts |= facts
            all_facts = set(f for f in all_facts if f.svar.function in o_funcs)
            # node_queue = chain(*itertools.izip_longest(*[n.add_facts(choices) for n in this_nodes]))
            # all_new_facts = set(chain(*[f for f in node_queue]))
            # for fact in sorted(all_facts, key=lambda f: self.cond_entropy(init, update({}, [f]), limit)):
            all_facts_with_entropy = [(f, self.entropy(init, update({}, [f]), limit)) for f in all_facts]
            for fact, H in sorted(all_facts_with_entropy, key=lambda (f,H): H):
                next = update(selected, [fact])
                tsize = len(self.compute_states(next, limit))
                if tsize <= limit and H < H_init - 0.0001:
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

    # def compute_flat_state(self, detstate):
    #     pstate = prob_state.ProbabilisticState(detstate.iteritems(), detstate.problem)
    #     def build_state(node):
    #         result = defaultdict(lambda: 0.0)
    #         p_total = 0.0
    #         # print node
    #         for val, (p, nodes, facts) in node.children.iteritems():
    #             p_total += p
    #             for svar, v in chain(facts.iteritems(), [(node.svar, val)]):
    #                 if svar in selected_facts and v in selected_facts[svar]:
    #                     # print "set:", svar, v
    #                     sfacts[svar_order[svar]] = v
    #             branch_states = {tuple(sfacts) : 1.0}
    #             # print map(str, sfacts)

    #             for n in nodes:
    #                 branch_states = multiply_states(branch_states, build_state(n))
    #                 if len(branch_states) > limit:
    #                     return branch_states

    #             # print node, val, len(branch_states)
    #             for bs, bp in branch_states.iteritems():
    #                 # print map(str, bs) , p*bp
    #                 result[bs] += p*bp

    #             if len(result) > limit:
    #                 return result
    #         if p_total < 1.0:
    #             result[undef] += 1.0 - p_total
                
    #         return result
            

    def compute_states(self, selected_facts, limit, order=None, filter_func=None):
        if not order:
            order = list(selected_facts.iterkeys())
        svar_order = dict((var, i) for i, var in enumerate(order))
        undef = (pddl.UNKNOWN,)*len(selected_facts)
        # print "build states", map(str, selected_facts)
        
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
            if not node.is_expanded():
                return result
            # print node
            for val, (p, nodes, facts) in node.children.iteritems():
                if filter_func and not filter_func(node, val, p, facts):
                    continue
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

                # print node, val, len(branch_states)
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

    def entropy(self, facts, condfacts, limit, filter_func=None):
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

        # print "entropy", map(str, relevant_facts)
        states = self.compute_states(relevant_facts, limit, order, filter_func)
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

class FlatDTPDDLOutput(task.PDDLOutput):
    # class RemoveProbTranslator(pddl.translators.Translator):
    #     def translate_problem(self, _problem):
    #         p2 = pddl.translators.Translator.translate_problem(self, _problem)
    #         p2.init = [i for i in p2.init if not isinstance(i, pddl.effects.ProbabilisticEffect)]
    #         return p2
        
    class FlatWriter(dtpddl.DTPDDLWriter):
        def __init__(self, pnodes):
            self.pnodes = pnodes

        def compute_states(self):
            def all_svars(node):
                return set(f.svar for f in node.all_facts())
            svars = reduce(lambda x,y: x|y, map(all_svars, self.pnodes), set())
            print map(str, svars)
            order = dict((var, i) for i, var in enumerate(svars))

            undef = (pddl.UNKNOWN,)*len(svars)
            # print "build states", map(str, selected_facts)

            def multiply_states(sset1, sset2):
                result = defaultdict(lambda: 0.0)
                # print "--------------------------"
                # for s, p in sset1.iteritems():
                #     print map(str,s), p
                # print
                # for s, p in sset2.iteritems():
                #     print map(str,s), p

                # print len(sset1), len(sset2)
                lost_prob = 0.0
                for (s1, p1), (s2, p2) in product(sset1.items(), sset2.items()):
                    res = []
                    for v1, v2 in itertools.izip(s1, s2):
                        if v1 == pddl.UNKNOWN:
                            res.append(v2)
                        else:
                            if v2 == pddl.UNKNOWN:
                                res.append(v1)
                            else:
                                lost_prob += p1*p2
                                res = None
                                break
                    if res:
                        result[tuple(res)] += p1*p2
                # print len(result)
                if lost_prob > 0.01:
                    print lost_prob
                    for res, p in result.iteritems():
                        result[res] *= 1/(1-lost_prob) - 0.01
                return result

            def build_state(node):
                result = defaultdict(lambda: 0.0)
                p_total = 0.0
                # print node
                for val, (p, nodes, facts) in node.children.iteritems():
                    p_total += p
                    sfacts = [pddl.UNKNOWN]*len(svars)
                    for svar, v in chain(facts.iteritems(), [(node.svar, val)]):
                        if svar in order:
                            sfacts[order[svar]] = v
                    branch_states = {tuple(sfacts) : 1.0}
                    # print map(str, sfacts)

                    for n in nodes:
                        branch_states = multiply_states(branch_states, build_state(n))

                    # print node, val, len(branch_states)
                    for bs, bp in branch_states.iteritems():
                        # print map(str, bs) , p*bp
                        result[bs] += p*bp

                if p_total < 1.0:
                    result[undef] += 1.0 - p_total

                return result

            states = {undef : 1.0}

            for n in self.pnodes:
                states = multiply_states(states, build_state(n))
            return states, order
            
        def write_flat_state(self):
            result = ["(probabilistic"]
            states, order = self.compute_states()
            svars = sorted(order.iterkeys(), key=lambda x: order[x])
            print "there are %d states" % len(states)
            ptot = 0.0
            for st, p in states.iteritems():
                if p < 0.00000001:
                    continue
                ptot += p
                elems = []
                for svar, val in itertools.izip(svars, st):
                    if val == pddl.UNKNOWN:
                        continue
                    if isinstance(svar.function, pddl.Predicate):
                        if val == pddl.TRUE:
                            elems.append("(%s %s)" % (svar.function.name, " ".join(a.name for a in svar.args)))
                    else:
                        elems.append("(%s %s %s)" % (svar.function.name, " ".join(a.name for a in svar.args), val.name))
                result.append("%.10f (and %s)" % (p, " ".join(elems)))
            print ptot
                
            return result + [")"]
            
        def write_init(self, inits):
            strings = []
            for i in inits:
                if isinstance(i, effects.ProbabilisticEffect):
                    pass
                else:
                    strings.append(self.write_literal(i))

            return self.section(":init", strings + self.write_flat_state())
        
    def __init__(self, pnodes):
        self.compiler = pddl.translators.ChainingTranslator( dtpddl.DTPDDLCompiler(), dtpddl.ProbADLCompiler())
        self.writer = FlatDTPDDLOutput.FlatWriter(pnodes)
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

