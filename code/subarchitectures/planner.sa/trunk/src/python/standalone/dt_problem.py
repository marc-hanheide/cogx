import time, math, itertools
from itertools import chain, product

from collections import defaultdict
from standalone import task, config, pddl, plans, relaxed_exploration, utils
from standalone.pddl import state, dtpddl, mapl, translators, visitors, effects

import standalone.globals as global_vars
import simplegraph
import partial_problem
import pstatenode, domain_query_graph

log = config.logger("dt")


def list_to_dict(facts):
    d = {}
    for svar, val in facts:
        if svar not in d:
            d[svar] = set([val])
        else:
            d[svar].add(val)
    return d

class DTProblem(object):
    def __init__(self, plan, pnodes, qgraph, failed_goals, prob_functions, global_rel_facts, domain):
        self.plan = plan
        self.pnodes = pnodes
        self.qgraph = qgraph
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
        # self.prob_functions |= set(r.function for r in self.dt_rules)
        self.global_relevant = global_rel_facts
        
        self.dtdomain = self.create_dt_domain(domain)
        log.debug("Time to DT creation: %.2f sec", global_vars.get_time())

        self.goal_generator = None

        # dom_str, prob_str = DTPDDLOutput().write(self.problem)
        # print "\n".join(dom_str)
        # print "\n".join(prob_str)
        
    def initialize(self, prob_state):
        domain_query_graph.check_time = 0.0
        self.state = prob_state
        self.detstate = prob_state.determinized_state(0.05, 0.95)
        
        def node_filter(node):
            for f in node.all_facts():
                if f.svar.function == dtpddl.selected:
                    continue
                if f.svar not in self.state or not self.state.is_det(f.svar):
                    return True
            return False

        #self.pnodes = filter(node_filter, self.pnodes)
            
        selected, relevant = self.reduce_state(global_vars.config.dt.max_state_size)
        self.selected_facts = set(selected)
        used_objects = set()
        for f in chain(selected, relevant):
            used_objects.update(f.svar.args)
            used_objects.update(f.svar.modal_args)
            used_objects.add(f.value)
        # for var, vals in facts.iteritems():
        #     for v in vals:
        #         add_objects.add(v)
        #         self.selected_facts.add(state.Fact(var, v))
        #         log.debug("selected: %s, %s", var, v)

        log.debug("Time to state reduction: %.2f sec", global_vars.get_time())
        log.debug("Node check time for state reduction: %.2f sec", domain_query_graph.check_time)
        domain_query_graph.check_time = 0.0
                
        # for o, mapping in self.get_observations(self.selected_facts):
        #     print "(%s %s)" % (o.name, " ".join(mapping.get(a,a).name for a in o.args))

        # used_objects = set()

        # o_actions = list(chain(*(self.get_observe_actions(f) for f in self.selected_facts)))
        # # for a, mapping in o_actions:
        # #     print "(%s %s)" % (a.name, " ".join(mapping.get(a,a).name for a in a.args))

        # t0 = time.time()
        # actions, explored_facts = relaxed_exploration.explore(self.domain.actions, set(), self.detstate, self.domain, o_actions)
        # log.debug("total time for exploration: %.2f", time.time()-t0)

        # for a, args in actions:
        #     used_objects |= set(args)
                
        def objects(svar, val):
            return set(svar.args + svar.modal_args + (val,))
                
        used_objects |= set(chain(*[pnode.full_args for pnode, level in self.assumptions]))
        log.debug("used_objects objects: %s", map(str, used_objects))

        used_objects |= self.domain.constants

        opt = "maximize"
        opt_func = pddl.FunctionTerm(pddl.dtpddl.reward, [])
        threshold = global_vars.config.uncertainty_threshold
        init = [f.to_init() for f in self.state.deterministic(threshold) if not f.value.is_instance_of(pddl.t_number) and (objects(*f) < used_objects)]
        # for f in self.state.deterministic():
        #     print f.to_init(),  (objects(*f) < used_objects)

        observable_facts = set(f for f in self.selected_facts if self.has_observations(f))

        # print "observable:", map(str, observable_facts)
        log.debug("Creating initial state: %.2f sec", global_vars.get_time())

        node_filter_facts = set(chain(self.selected_facts, observable_facts))

        def check_node(n, parent_facts):
            return n.check_node(node_filter_facts, self.detstate, self.qgraph, parent_facts=parent_facts)
        
        for n in self.pnodes:
            peff, obs = n.to_init(self.selected_facts, observable_facts, filter_fn=check_node)
            if obs and peff:
                # print obs , peff.pddl_str()
                used_objects |= set(visitors.visit(peff, pddl.visitors.collect_constants, [])) - self.domain.constants
                init.append(peff)
                
        log.debug("Initial state created: %.2f sec", global_vars.get_time())
        log.debug("Node check time for state creation: %.2f sec", domain_query_graph.check_time)
        domain_query_graph.check_time = 0.0
                
        self.problem = pddl.Problem("cogxtask", used_objects - self.domain.constants, init, None, self.dtdomain, opt, opt_func)
        self.problem.goal = pddl.Conjunction([])

        self.abstract_state = pddl.prob_state.ProbabilisticState.from_problem(self.problem)

        self.goal_generator = DefaultGoalFactory(self.goals, self.selected_facts, self.assumptions, self.abstract_state, self.dtdomain)
        self.goal_generator.set_dt_problem(self)

        self.dtdomain.actions += list(filter(None, self.goal_generator.get_goal_actions()))
        self.dtdomain.name2action = None
        
        log.debug("Time to DT initialisation: %.2f sec", global_vars.get_time())
        log.debug("Node check time for goal creation: %.2f sec", domain_query_graph.check_time)
        self.write_dt_input("dtdomain.dtpddl", "dtproblem.dtpddl")

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

    def get_observe_actions(self, fact):
        for o, mapping in self.get_observations(fact):
            for e in o.execution:
                action = e.action
                o_a_mapping = dict(zip(e.args, action.args))
                a_mapping = {}
                for oarg, val in mapping.iteritems():
                    if oarg in o_a_mapping:
                        a_mapping[o_a_mapping[oarg]] = val
                yield action, a_mapping
        # for a in self.domain.actions:
        #     #try mapl-sensors
        #     sensors = None
        #     try:
        #         sensors = a.sensors
        #     except:
        #         pass
        #     for s in sensors:
        #         term = s.get_term()
        #         mapping =  dict(zip(term.args, fact.svar.args))
        #         yield action, mapping

    def has_observations(self, fact):
        for _ in self.get_observations(fact):
            return True
        # for a in self.domain.actions:
        #     sensors = None
        #     try:
        #         sensors = a.sensors
        #     except:
        #         pass
        #     for s in sensors:
        #         if s.get_term().function == fact.svar.function:
        #             return True
        return False

    def get_observations(self, fact):
        try:
            observations = self.observations_by_function.get(fact.svar.function, [])
        except:
            @visitors.collect
            def atom_visitor(elem, results):
                if isinstance(elem, pddl.LiteralCondition):
                    if elem.predicate != dtpddl.observed:
                        return [elem]

            @visitors.collect
            def effect_visitor(elem, results):
                if isinstance(elem, pddl.ConditionalEffect):
                    return elem.condition.visit(atom_visitor)

            self.observations_by_function = defaultdict(list)

            for o in self.domain.observe:
                sensable_atoms = []
                if o.precondition:
                    sensable_atoms += o.precondition.visit(atom_visitor)
                sensable_atoms += o.effect.visit(effect_visitor)
                for lit in sensable_atoms:
                    function = pddl.utils.get_function(lit)
                    self.observations_by_function[function].append((o, lit))
                    
            observations = self.observations_by_function.get(fact.svar.function, [])
                    

        for o, lit in observations:
            mapping = fact.match_literal(lit)
            if mapping is not None:
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
        observe_effects = translators.Translator.get_annotations(self.domain).get('observe_effects', {})
        sense_effects = set()
            
        for senses in observe_effects.itervalues():
            sense_effects |= senses
        for a in self.domain.actions:
            sense_effects |= set(a.sensors)
            
        sensed_functions = set(s.get_term().function for s in sense_effects)
        changed = True
        while changed:
            changed = False
            for rule in self.domain.dt_rules:
                if not rule.deps < sensed_functions:
                    if any(f in sensed_functions for f, _ in rule.variables):
                        sensed_functions |= rule.deps
                        changed = True

        # print map(str, sensed_functions)

        def get_real_sensor(action, effect):
            if effect.svar.modality not in (pddl.mapl.knowledge, pddl.mapl.direct_knowledge):
                return None
            
            if action.name in observe_effects:
                sensors = observe_effects[action.name]
            else:
                try:
                    action = self.domain.get_action(action.name)
                    sensors = action.sensors
                except:
                    sensors = []
                    
            for s in sensors:
                if s.get_term().function == effect.svar.function:
                    return s
            return None
            
        def triggers_dt(pnode):
            # print "checking", pnode

            #condition 1: actions must have a knowledge effect corresponding to a MAPL sensor
            k_effects = set()
            for s, eff in ((get_real_sensor(pnode.action, e), e) for e in pnode.effects):
                if s:
                    k_effects.add(eff.svar)
            # print "knowledge effects:", map(str, k_effects)

            #condition 2: k-effect must be reachable from DT observations
            k_effects = set(f for f in k_effects if f.function in sensed_functions)
            # print "DT k-effects:", map(str, k_effects)
            if not k_effects:
                return False
            
            #condition 3: another action (or the goal) must depend on the k-effect
            for succ in plan.successors_iter(pnode):
                for link in plan[pnode][succ].itervalues():
                    # print "checking link to %s: svar=%s" % (str(succ), str(link['svar']))
                    if link['svar'] in k_effects:
                        # print "ok"
                        return True
            return False
            

        def find_restrictions(pnode, level):
            result = {}
            for pred in plan.predecessors_iter(pnode, 'depends'):
                restr = find_restrictions(pred, level+1)
                if pred.action.name.startswith("__commit-"):
                    result[pred] = max(result.get(pred, -1), level+1)
                    # others.append((pred, level+1))
                    # return [(pred, level+1)] + restr
                for pn, l in restr.iteritems():
                    result[pn] = max(result.get(pn, -1), l)
            return result

        goal_facts = set()
        assumptions = []
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            # if pnode.action.name in observe_actions:
            # print "check pnode:", pnode
            if triggers_dt(pnode):
                #TODO: only add an action if the observe effect supports a later action
                for fact in pnode.effects:
                    if fact.svar.function not in (pddl.builtin.total_cost, ):
                        goal_facts.add(fact)
                self.subplan_actions.append(pnode)
                # print pnode
                assumptions += [(pnode,0)] + [(pn, l) for pn, l in find_restrictions(pnode,0).iteritems()]
                log.debug("assumptions: %s", ", ".join("%s/%d" % (str(a),l) for a,l in assumptions))
                break
            
            # if pnode.action.name not in observe_actions and self.subplan_actions:
            #     break
        
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
            # print ground_fact
            # print map(str, self.selected_facts)
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

    def get_dt_results(self, action):
        return self.goal_generator.committments_from_action(action)

    def is_goal_action(self, action_name):
        return self.goal_generator.is_goal_action(action_name)

    def create_dt_domain(self, dom):
        dtdomain = dom.copy()
        dtdomain.name = "dt-%s" % dom.name
        dtdomain.annotations =  pddl.translators.Translator.get_annotations(dom)
        return dtdomain

    def reduce_state2(self, limit):
        o_funcs = self.observable_functions()

        def is_reachable(node, fact, parent_facts = {}):
            """Check whether a fact can be generated by a given node"""
            if isinstance(node, list):
                return any(is_reachable(n, fact) for n in node)
            else:
                return node.check_node([fact], self.detstate, self.qgraph, parent_facts=parent_facts)
        
        def get_reachable_facts(node, facts, parent_facts = {}):
            """Return the subset of a set of facts that can be generated by a given node"""
            def collect_fn(it):
                # it = list(it)
                # for res, f in it:
                #     print "  ", f, res
                return list(set(f for res, f in it if res))
            return node.check_node(facts, self.detstate, self.qgraph, eval_fn=collect_fn, parent_facts=parent_facts)

        def get_observable_facts(node):
            """Return all observable facts that cen be generated by a given node"""
            # facts = get_reachable_facts(node, facts)
            # if not facts:
            #     # print node
            #     return []
            result = []
            for val, (p, nodes, nfacts) in node.children.iteritems():
                factgen = (state.Fact(s,v) for s,v in chain(nfacts.iteritems(), [(node.svar, val,)]))
                for f in factgen:
                    if self.has_observations(f):
                        result.append(f)
                for n in nodes:
                    result += get_observable_facts(n)
            return result

        nonstatic = self.domain.get_nonstatic_functions() | set([mapl.commit, mapl.knowledge, mapl.direct_knowledge, mapl.indomain, mapl.i_indomain])
        factcache = {}

        check_time = [0]
        check_count = [0,0,0]
        
        # print "nonstatic:", map(str, nonstatic)
        def make_check_fn(init_fact):
            def exploration_check_fn(fact):
                t0 = time.time()
                try:
                    if fact.svar.modality == pddl.mapl.commit:
                        fact = state.Fact(fact.svar.nonmodal(), fact.svar.modal_args[0])

                    if init_fact.svar == fact.svar and ((init_fact.value != fact.value) ^ fact.negated()):
                        # print "constraint %s violated: %s" % (init_fact, fact)
                        return relaxed_exploration.FACT_STATICALLY_FALSE

                    if (fact.svar.function in nonstatic and not fact.svar.modality) or fact.svar.modality in nonstatic:
                        if fact in self.detstate:
                            return relaxed_exploration.FACT_CURRENTLY_TRUE
                        else:
                            return relaxed_exploration.FACT_CURRENTLY_FALSE
                    else:
                        if fact in self.detstate:
                            return relaxed_exploration.FACT_STATICALLY_TRUE                        
                        elif fact.svar in self.detstate:
                            return relaxed_exploration.FACT_STATICALLY_FALSE
                        elif fact.svar.function not in self.prob_functions:
                            return relaxed_exploration.FACT_STATICALLY_FALSE
                            
                        # print "test reachability:", fact
                        if fact not in factcache:
                            check_count[1] += 1
                            if is_reachable(self.pnodes, fact):
                                # print fact, "reachable"
                                factcache[fact] = relaxed_exploration.FACT_CURRENTLY_TRUE
                            else:
                                # print fact, "unreachable"
                                factcache[fact] = relaxed_exploration.FACT_STATICALLY_FALSE
                            # print "%s reachable: %s"x % (fact, factcache[fact])
                        check_count[2] += 1
                        return factcache[fact]
                finally:
                    check_time[0] += time.time() - t0
                    check_count[0] += 1
            return exploration_check_fn
        
        def get_dependent_facts(fact):
            goal_facts = []
            check_time[0] = 0.0
            check_count[:] = [0, 0, 0]
            cfact = state.Fact(fact.svar.as_modality(mapl.commit, [fact.value]), pddl.TRUE)
            relaxed_exploration.cache.set_fact(fact)
            relaxed_exploration.cache.set_fact(cfact)
            for agent_obj in self.state.problem.get_all_objects(mapl.t_planning_agent):
                goal_facts.append(state.Fact(fact.svar.as_modality(mapl.direct_knowledge, [agent_obj]), pddl.TRUE))
            # print map(str, goal_facts)
            actions, explored_facts = relaxed_exploration.explore(self.domain.actions, set(goal_facts), self.detstate, self.domain, prob_functions = self.prob_functions, check_fn=make_check_fn(fact))
            relaxed_exploration.cache.unset_fact(fact)
            relaxed_exploration.cache.unset_fact(cfact)
            # print "total check time: %.3f" % check_time[0]
            # print "number of complex queries: %d (cached: %d), total number of queries: %d" % (check_count[1], (check_count[2] -check_count[1]),  check_count[0])
            explored_facts = [state.Fact(f.svar.nonmodal(), f.svar.modal_args[0]) if f.svar.modality == mapl.commit else f for f in explored_facts]
            return actions, explored_facts

        relaxed_exploration.init_cache(self.detstate)
                    
        choices = self.extract_choices()
        assumptions = []
        for svar, (val, _) in choices.iteritems():
            if svar.function in self.prob_functions and svar not in self.detstate:
                assumptions.append(state.Fact(svar, val))
        log.debug("Facts from assumptions: %s", map(str, assumptions))

        #Get observable facts which are possibly correlated to out assumptions
        t0 = time.time()
        candidates = set()
        for n in self.pnodes:
            #TODO: this is a hack to get reasonable results for avs/interactive dora
            #TODO: in general this condition may be too strict
            reachable = get_reachable_facts(n, assumptions)
            # print n, "reachable:", map(str, reachable)
            if len(reachable) == len(assumptions):
                facts = get_observable_facts(n)
                candidates |= set(facts)
            #     print "ok:", map(str, facts)
            # else:
            #     print "reject"
        log.debug("initial candidates: %s", map(str, candidates))
        # print "Time for candidate generation: %.3f" % (time.time() -t0)
        log.debug("Time to candidate generation: %.2f sec", global_vars.get_time())

        H_init = self.entropy(assumptions, [], limit)
        log.debug("H(A) = %.3f", H_init)
        #Get facts on which these candidates depend using a relaxed plangraph
        dependencies = {}
        observable_dependencies = {}
        candidate_sets = []
        for f in candidates:
            plan, depends = get_dependent_facts(f)
            if not plan:
                log.debug("no relaxed plan to observe %s:", str(f))
                continue
                
            o_depends = [g for g in depends if self.has_observations(g) and g != f]
            log.debug("observable preconditions of %s:", str(f))
            log.debug(", ".join(map(str, o_depends)))
            observable_dependencies[f] = set(o_depends)
            dependencies[f] = depends
            #TODO: this is not very generic, only works for one mutex set of dependencies
            if o_depends:
                for of in o_depends:
                    candidate_sets.append((f, of))
            else:
                candidate_sets.append((f,))
                
        log.debug("Time to dependency detection: %.2f sec", global_vars.get_time())

        entropy_cache = {}
        def calc_entropy(facts):
            if facts not in entropy_cache:
                entropy_cache[facts] = self.entropy(assumptions, facts, limit)
            return entropy_cache[facts]

        probability_cache = {}
        def calc_probabilities(ofacts, source_fact):
            if source_fact not in probability_cache:
                states = self.compute_states(list_to_dict(observable_dependencies[source_fact]), limit)
                probability_cache[source_fact] = states
            else:
                states = probability_cache[source_fact]
            return dict((f, states[(f.value,)]) for f in ofacts)
                                    
        def get_entropy(fset, prev_facts=[], prev_o=[], debug=False):
            if len(fset) == 1:
                fact = fset[0]
                assert not observable_dependencies[fact]
                H = calc_entropy((fact,) + tuple(prev_facts))
                if debug:
                    print "H(A|%s) =  %.3f" % (fact, H)
                return H
            else:
                fact, ofact = fset
                used__ofacts = [f for f in prev_o if f in observable_dependencies[fact]]
                if ofact not in used__ofacts:
                    used__ofacts.append(ofact)
                
                o_probs = calc_probabilities(used__ofacts, fact)
                p = sum(o_probs[f] for f in used__ofacts )
                if debug:
                    print "P(K %s|prev+%s) = %.3f" % (fact, ofact, p)
                H_prev = calc_entropy(tuple(p for p in prev_facts if p.svar != fact.svar))
                H_new = calc_entropy((fact,) + tuple(prev_facts))
                H_tot = p * H_new + (1-p) * H_prev
                if debug:
                    print "H(A|%s, %s) = %.3f * %.3f + (1-p) * %.3f =  %.3f" % (fact, ofact, p, H_new, H_prev, H_tot)
                return H_tot
                
            
        # for c in candidate_sets:
        #     get_entropy(c, debug=True)
            
        candidate_queue = sorted(candidate_sets, key=lambda f: get_entropy(f))
        used = []
        o_used = []
        while candidate_queue:
            c = candidate_queue.pop(0)
            if len(c) == 1:
                fact = c[0]
                ofact = None
                log.debug("Use %s", fact)
                # print "Use %s with H=%.3f" % (fact, get_entropy(c, used, debug=True))
                # used.append(fact)
            else:
                fact, ofact = c
                log.debug("Use (%s, %s)", fact, ofact)
                # print "Use (%s, %s) with H=%.3f" % (fact, ofact, get_entropy(c, used, o_used, debug=True))
                # used.append(fact)
                # o_used.append(ofact)

            fdict = list_to_dict(chain(assumptions, used, o_used, c))
            def fact_filter(svar, val=None):
                return svar in fdict
            states = None
            with utils.log_time("creating states", log.debug):
                states = self.compute_states(fdict, limit)#, filter_func=fact_filter)
            # for s in states.iterkeys():
            #     print map(str, s)
            # print ["%s=%s" %(k, map(str,v)) for k,v in fdict.iteritems()]
            # for f, p in states.iteritems():
            #     print "P(%s) = %.3f" % (map(str, f), p)
            tsize = len(states)
            log.debug("new state size: %d" % tsize)
            if tsize <= limit:
                used.append(fact)
                if ofact and ofact not in o_used:
                    o_used.append(ofact)
                    
                candidate_queue = sorted(candidate_queue, key=lambda f: get_entropy(f, used, o_used))
            else:
                log.debug("skipped")
            
        selected = assumptions + used + o_used
        relevant_facts = set()
        for f in selected:
            if f in dependencies:
                relevant_facts |= set(dependencies[f])
        return selected, relevant_facts
        
        

    def reduce_state(self, limit):
        return self.reduce_state2(limit)
    
        levels = defaultdict(lambda: -1)
        def get_level(node, level):
            cval, clevel = choices.get(node.svar, (None, -1))
            log.debug("get level: %s, %s", str(node), str(cval))
            if cval and levels[node] < level and cval in node.children:
                levels[node] = level
                node.unify_branches()
                p, nodes, facts = node.children[cval]
                for svar, val in facts.iteritems():
                    choices[svar] = (val, clevel)
                    # cfacts.update(facts)
                for n in nodes:
                    get_level(n, level+1)
                    
        # def get_candidates(node, chosen_facts):
        #     if isinstance(n, pstatenode.LazyPNode):
                
        #         rule = n.rule
        #         args = [n.mapping.get(a,a) for a in rule.args]
        #         # print "---"
        #         for fact in node_filter_facts:
        #             # print "checking %s against %s" % (str(fact), str(n))
        #             rel = self.qgraph.query_reachability(rule, args, fact, self.state.problem, self.detstate)
        #             if rel:
        #                 return True
        #         # print "reject", n
        #         return False
        #     else:
                
        #     return True
            

        def total_size(nodes, facts=None):
            return reduce(lambda x,y: x*y, [n.size(facts) for n in nodes], 1)
        
        def update(d, it):
            d = dict((v,set(s)) for v,s in d.iteritems())
            for svar, val in it:
                s = d.setdefault(svar, set())
                s.add(val)
            return d

        def get_dependent_facts(fact):
            o_actions = list(self.get_observe_actions(fact))
            for a, m in o_actions:
                print a.name, ", ".join(str(m.get(arg, None)) for arg in a.args)
            actions, explored_facts = relaxed_exploration.explore(self.domain.actions, set(), self.detstate, self.domain, o_actions)
            for f in explored_facts:
                if f.modality == mapl.commit:
                    yield state.Fact(f.nonmodal(), f.modal_args[0])
            

        choices = self.extract_choices()
        log.debug("initial choices: %s", map(str, choices))
        o_funcs = self.observable_functions()

        for n in self.pnodes:
            # print n
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
                    get_dependent_facts(state.Fact(svar, val))
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
            all_facts = set(f for f in all_facts if self.has_observations(f) and f.value not in selected.get(f.svar, set()))
            # print map(str, all_facts)
            # node_queue = chain(*itertools.izip_longest(*[n.add_facts(choices) for n in this_nodes]))
            # all_new_facts = set(chain(*[f for f in node_queue]))
            # for fact in sorted(all_facts, key=lambda f: self.cond_entropy(init, update({}, [f]), limit)):
            all_facts_with_entropy = [(f, self.entropy(init, update({}, [f]), limit)) for f in all_facts]
            for fact, H in sorted(all_facts_with_entropy, key=lambda (f,H): H):

                get_dependent_facts(fact)
                    
                next = update(selected, [fact])
                def fact_filter(svar, val=None):
                    return svar in next
                    
                tsize = len(self.compute_states(next, limit, filter_func=fact_filter))
                log.debug("candidate: %s (%d, %.3f, %.3f)", str(fact), tsize, H, H < H_init)
                if tsize <= limit and H < H_init - 0.0001:
                    log.debug("added: %s (%d, %.3f)", str(fact), tsize, H)
                    selected = next
                else:
                    log.debug("skipped: %s (%d, %.3f)", str(fact), tsize, H)
                    pass

        return selected
            

    def compute_states(self, selected_facts, limit, order=None, filter_func=None):
        if not order:
            order = list(selected_facts.iterkeys())
        try:
            cache = {}#self.state_cache
        except:
            self.state_cache = {}
            cache = self.state_cache
            
        svar_order = dict((var, i) for i, var in enumerate(order))
        undef = (pddl.UNKNOWN,)*len(selected_facts)
        factlist = list(chain(*([state.Fact(svar, val) for val in values] for svar, values in selected_facts.iteritems())))
        # print "build states", map(str, selected_facts)

        # tsum = [0.0]
        
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
        
        def check_node(n, facts, parent_facts={}):
            """Return the subset of a set of facts that can be generated by a given node"""
            def collect_fn(it):
                # it = list(it)
                # for res, f in it:
                #     assert isinstance(f, state.Fact)
                return [f for res, f in it if res]
            rel_facts = n.check_node(facts, self.detstate, self.qgraph, eval_fn=collect_fn, parent_facts=parent_facts)
            # if any(f.svar.function.name == "visible_from" for f in rel_facts):
            #     print n#, map(str, facts)
            #     print "              ", map(str, rel_facts)
            return len(rel_facts) > 0, rel_facts
                
        def build_state(node, rel_facts, parent_facts={}):
            key = (node, frozenset(rel_facts))
            if key in cache:
                # print "early hit:", node, map(str, rel_facts)
                return cache[key]
            
            result = defaultdict(lambda: 0.0)
            p_total = 0.0
            
            if filter_func and not filter_func(node.svar):
                # print len(result), "(break)"
                cache[key] = {undef : 1.0}
                return {undef : 1.0}
            # oldrel = rel_facts
            rel, rel_facts = check_node(node, rel_facts, parent_facts)
            if not rel:
                # print ["%s=%s" % (k,v) for k,v in parent_facts.iteritems())
                # print node, map(str, oldrel)
                cache[key] = {undef : 1.0}
                return {undef : 1.0}
            key = (node, frozenset(rel_facts))
            if key in cache:
                # print "late hit:", node, map(str, rel_facts)
                return cache[key]
            # print " ---- no hit", node, map(str, rel_facts)
            
            # print "n:", node
            for val, (p, nodes, facts) in node.children.iteritems():
                # print val
                # if filter_func and not filter_func(node, val, p, facts):
                if filter_func and not filter_func(node.svar, val):
                    continue
                new_parent_facts = parent_facts.copy()
                p_total += p
                # print val, p, p_total
                sfacts = [pddl.UNKNOWN]*len(selected_facts)
                for svar, v in chain(facts.iteritems(), [(node.svar, val)]):
                    new_parent_facts[svar] = v
                    if svar in selected_facts and v in selected_facts[svar]:
                        # print "set:", svar, v
                        sfacts[svar_order[svar]] = v
                branch_states = {tuple(sfacts) : 1.0}
                # print map(str, sfacts)

                for n in nodes:
                    branch_states = multiply_states(branch_states, build_state(n, rel_facts, new_parent_facts))
                    if len(branch_states) > limit:
                        # print len(branch_states)
                        return branch_states

                # print node, val, len(branch_states)
                for bs, bp in branch_states.iteritems():
                    # print map(str, bs) , p*bp
                    result[bs] += p*bp

                if len(result) > limit:
                    # print len(result)
                    return result
            if p_total < 1.0:
                result[undef] += 1.0 - p_total
                
            # print len(result)
            cache[key] = result
            return result

        states = {undef : 1.0}

        for n in self.pnodes:
            states = multiply_states(states, build_state(n, factlist))
            if len(states) > limit:
                break
        # print "total check time: %.3f" % tsum[0]
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
        facts = list_to_dict(facts)
        condfacts = list_to_dict(condfacts)

        def fact_filter(svar, val=None):
            # print "filter:", svar, val, " = ", svar in facts or svar in condfacts
            return svar in facts or svar in condfacts

        if filter_func is None:
            filter_func = fact_filter
        
        order = list(svar for svar in facts.iterkeys() if svar not in condfacts) + list(condfacts.iterkeys())
        svar_order = dict((var, i) for i, var in enumerate(order))
        cf_svars = order[-len(condfacts):] if condfacts else []
        
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
                cf = tuple(val if val in condfacts[svar] else pddl.UNKNOWN for val, svar in zip(cf, cf_svars))
                cfdict[cf] += p
        # for f, p in cfdict.iteritems():
        #     print "P(%s) = %.3f" % (map(str, f), p)
        # print "\n"
        # for f, p in states.iteritems():
        #     print "P(%s) = %.3f" % (map(str, f), p)
            
        H = 0
        for s, p in states.iteritems():
            cf = s[-len(condfacts):]
            cf = tuple(val if val in condfacts[svar] else pddl.UNKNOWN for val, svar in zip(cf, cf_svars))
            # print map(str, s), "  ", map(str, cf)
            if p <= 0.000000001:
                continue
            if condfacts:
                # print "+ %.5f * log(%.5f/%.5f) = %.5f" % (p, cfdict[cf], p, p * math.log(cfdict[cf]/p,2) )
                H += p * math.log(cfdict[cf]/p,2)
            else:
                # print "- %.5f * log(%.5f) = %.5f" % (p, p, -p * math.log(p,2) )
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

class GoalActionFactory(object):
    def __init__(self, goals, relevant, assumptions, state, domain):
        self.config = global_vars.config.dt
        self.goals = goals
        self.relevant = relevant
        self.assumptions = assumptions
        self.state = state
        self.domain = domain
        self.base_reward = self.config.confirm_score
        self.confirm_dict = {}
        self.disconfirm_dict = {}
        self.goal_actions = set()

    def set_base_reward(self, reward):
        self.base_reward = reward

    def committments_from_action(self, action):
        if action.name in self.confirm_dict:
            return self.confirm_dict[action.name], None
        if action.name in self.disconfirm_dict:
            return None, self.disconfirm_dict[action.name]
        return None, None

    def is_goal_action(self, action_name):
        return action_name in self.goal_actions
    
    def clear_state_effect(self):
        # effs = [f.to_effect() for f in chain(selected, changed_facts) ]
        # return pddl.ConjunctiveEffect([])
        return pddl.ConjunctiveEffect([])
    
    def get_goal_facts(self):
        for fact in self.goals:
            if fact.svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                svar = fact.svar.nonmodal()
            else:
                log.debug("Goal not yet supported: %s", str(fact))
                continue

            rel_facts = (f for f in self.relevant if f.svar == svar)
            for f in rel_facts:
                if self.state.is_det(svar):
                    p = 1.0 if self.state[svar] == f.value else 0.0
                else:
                    p = self.state[svar][f.value]
                    
                if p == 0.0:
                    continue
                
                yield f, p

    def get_disconfirm_facts(self):
        disconfirm = set()
        for pnode, level in self.assumptions:
            for svar, val in pnode.effects:
                if svar.modality == mapl.commit:
                    nmvar = svar.nonmodal()
                    disconfirm.add(state.Fact(nmvar, svar.modal_args[0]))
                    break # only one disconfirm per node

        for fact in disconfirm:
            if self.state.is_det(fact.svar):
                continue # no use in disconfirming known facts
            p = self.state[fact.svar][fact.value]

            yield fact, p

    def get_commit_reward(self, fact, p):
        reward = self.base_reward
        penalty = -reward * (p)/(1-p) if p != 1.0 else 0

        return reward, penalty
    
    def get_disconfirm_reward(self, fact, p):
        reward = self.base_reward - 20
        penalty = -reward * (1-p)/p

        return reward, penalty
    
    def create_commit_action(self, fact, p):
        reward, penalty = self.get_commit_reward(fact, p)

        for a in fact.all_args():
            self.domain.add_constant(a)
        
        name = "commit-%s-%s-%s" % (fact.svar.function.name, "-".join(a.name for a in fact.svar.get_args()), fact.value.name)
        self.confirm_dict[name] = fact
        a = pddl.Action(name, [], None, None, self.domain)
        b = pddl.builder.Builder(a)
        term = pddl.Term(fact.svar.function, fact.svar.get_args())

        #a.precondition = b.cond('and', ('not', (mapl.committed, [term])))
        a.precondition = b.cond('not', ('done', ))
        # commit_effect = b.effect(mapl.committed, [term])
        reward_effect = b('when', ('=', term, fact.value), ('assign', ('reward',), reward ))
        penalty_effect = b('when', ('not', ('=', term, fact.value)), ('assign', ('reward',), penalty))
        done_effect = b.effect('done')
        a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, self.clear_state_effect())

        self.goal_actions.add(a.name)
        return a

    def create_disconfirm_action(self, fact, p):
        reward, penalty = self.get_disconfirm_reward(fact, p)

        if reward <= 0:
            return

        for a in fact.all_args():
            self.domain.add_constant(a)
        
        name = "disconfirm-%s-%s" % (fact.svar.function.name, "-".join(a.name for a in fact.svar.get_args()))
        self.disconfirm_dict[name] = fact
        a = pddl.Action(name, [], None, None, self.domain)
        b = pddl.builder.Builder(a)
        term = pddl.Term(fact.svar.function, fact.svar.get_args())

        a.precondition = b.cond('not', ('done', ))
            
        reward_effect = b('when', ('not', ('=', term, fact.value)), ('assign', ('reward',), reward))
        penalty_effect = b('when', ('=', term, fact.value), ('assign', ('reward',), penalty))
        done_effect = b.effect('done')
        a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, self.clear_state_effect())

        self.goal_actions.add(a.name)
        return a
        
    def create_cancel_action(self):
        a = pddl.Action("cancel", [], None, None, self.domain)
        b = pddl.builder.Builder(a)
        a.precondition = b.cond('not', ('done', ))
        a.effect = b.effect('and', ('done', ), ('assign', ('reward',), 0), self.clear_state_effect())
        self.goal_actions.add(a.name)
        return[a] 

    def get_goal_actions(self):
        cancels = self.create_cancel_action()
        goals = (self.create_commit_action(f,p) for f,p in self.get_goal_facts())
        disconfirms = (self.create_disconfirm_action(f,p) for f,p in self.get_disconfirm_facts())
        return chain(cancels, goals, disconfirms)

class DefaultGoalFactory(GoalActionFactory):
    def set_fail_counts(self, counts):
        self.fail_counts = counts

    def set_dt_problem(self, dt):
        self.dt_problem = dt

    # def calc_disconfirm_gain(self, dis_fact, dep_var):
    #     def node_filter(node, val, p, facts):
    #         for svar, v in chain([(node.svar, val)], facts.iteritems()):
    #             if svar == dis_fact.svar and v == dis_fact.value:
    #                 return False
    #         return True

    #     if self.state.is_det(dep_var):
    #         return 0

    #     relevant_facts = {dep_var : self.global_relevant[dep_var]}
    #     H = -sum(p*math.log(p,2) for p in self.state[dep_var].itervalues() if p > 0)
    #     cH = self.entropy(relevant_facts, {}, 100, node_filter)
    #     # print "H(%s) - H(.|!%s) = %.2f - %.2f =  %.2f" % (str(dep_var), str(dis_fact), H, cH, H - cH)
    #     return max(H - cH, 0)
        
    def entropy_of_goal(self, dis_fact, goal_facts):
        def state_filter(svar, val=None):
            # if svar in goal_dict:
            #     return True
            if dis_fact and svar == dis_fact.svar and val == dis_fact.value:
                # print "reject state:", svar, val
                return False
            return True

        H = self.dt_problem.entropy(goal_facts, [], 100, state_filter)
        # print "H(%s) - H(.|!%s) = %.2f - %.2f =  %.2f" % (str(dep_var), str(dis_fact), H, cH, H - cH)
        return H

    def get_disconfirm_reward(self, fact, p):
        H = self.entropy_of_goal(fact, self.goal_facts)
        if self.H_goal > 0:
            reward_factor = 1 - H/max(H, self.H_goal)
        else:
            reward_factor = 0
            
        reward = float(self.base_reward - 20) * reward_factor
        penalty = -reward * (1-p)/p

        return reward, penalty
    
    def get_goal_actions(self):
        cancels = self.create_cancel_action()
        goals = (self.create_commit_action(f,p) for f,p in self.get_goal_facts())

        # common precalculations
        self.goal_facts = set(f for f,_ in self.get_goal_facts())
        # self.goal_p = sum(self.state.prob(f.svar, f.value) for f in goal_facts)
        self.H_goal = self.entropy_of_goal(None, self.goal_facts)
        
        disconfirms = (self.create_disconfirm_action(f,p) for f,p in self.get_disconfirm_facts())
        return chain(cancels, goals, disconfirms)
    
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


    # def create_goal_actions(self, goals, fail_counts, selected, changed_facts, domain):
    #     commit_actions = []
    #     dt_conf = global_vars.config.dt

    #     def clear_state_effect():
    #         effs = [f.to_effect() for f in chain(selected, changed_facts) ]
    #         return pddl.ConjunctiveEffect([])

    #     for f in chain(selected, changed_facts):
    #         for o in f.svar.get_args():
    #             domain.add_constant(o)
    #         domain.add_constant(f.value)

    #     # confirm_score = global_vars.config.dt.confirm_score
    #     # failure_multiplier = global_vars.config.dt.failure_multiplier

    #     a = pddl.Action("cancel", [], None, None, domain)
    #     b = pddl.builder.Builder(a)
    #     a.precondition = b.cond('not', ('done', ))
    #     a.effect = b.effect('and', ('done', ), ('assign', ('reward',), 0), clear_state_effect())
    #     commit_actions.append(a) # comment out to disable the cancel action

    #     self.confirm_dict = {}
    #     self.disconfirm_dict = {}
    #     goal_facts = set()
        
    #     for fact in goals:
    #         if fact.svar.modality in (mapl.knowledge, mapl.direct_knowledge):
    #             svar = fact.svar.nonmodal()
    #         else:
    #             log.debug("Goal not yet supported: %s", str(fact))
    #             continue

    #         #assert svar in self.global_relevant
    #         mult = dt_conf.failure_multiplier**fail_counts[fact]

    #         term = pddl.Term(svar.function, svar.get_args())
    #         for o in svar.get_args():
    #             domain.add_constant(o)

    #         rel_facts = [f for f in selected if f.svar == svar]
    #         for f in rel_facts:
    #             #if f.value not in self.global_relevant[f.svar]:
    #             #    continue
    #             if isinstance(self.abstract_state[svar], pddl.TypedObject):
    #                 p = 1.0 if self.abstract_state[svar] == f.value else 0.0
    #             else:
    #                 p = self.abstract_state[svar][f.value]
                    
    #             if p == 0.0:
    #                 continue

    #             if dt_conf.commitment_mode in ("conf", "all"):
    #                 reward = dt_conf.total_reward - self.remaining_costs
    #                 penalty = -dt_conf.total_reward - self.remaining_costs
    #             else:
    #                 reward = dt_conf.confirm_score * mult
    #                 penalty = -reward * (p)/(1-p) if p != 1.0 else 0

    #             goal_facts.add(f)
                                    
    #             #val = pddl.Parameter("?val", svar.function.type)
    #             name = "commit-%s-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()), f.value.name)
    #             self.confirm_dict[name] = f
    #             a = pddl.Action(name, [], None, None, domain)
    #             b = pddl.builder.Builder(a)

    #             #a.precondition = b.cond('and', ('not', (mapl.committed, [term])))
    #             a.precondition = b.cond('not', ('done', ))
    #             commit_effect = b.effect(mapl.committed, [term])
    #             reward_effect = b('when', ('=', term, f.value), ('assign', ('reward',), reward ))
    #             penalty_effect = b('when', ('not', ('=', term, f.value)), ('assign', ('reward',), penalty))
    #             done_effect = b.effect('done')
    #             a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, clear_state_effect())
            
    #             commit_actions.append(a)

    #     max_level = 0
    #     disconfirm = []
    #     levels = {}
    #     for pnode, level in self.assumptions:
    #         for svar, val in pnode.effects:
    #             if svar.modality == mapl.commit:
    #                 max_level = max(max_level, level)
    #                 nmvar = svar.nonmodal()
    #                 #if nmvar not in goals:
    #                 disconfirm.append((nmvar, svar.modal_args[0], level))
    #                 levels[(nmvar, svar.modal_args[0])] = max(levels.get((nmvar, svar.modal_args[0]), -1), level)
    #                 break # only one disconfirm per node

    #     if not disconfirm:
    #         return commit_actions

    #     disconfirm = set((svar, val) for svar, val, _ in disconfirm)

    #     # goal proposition could be regarded as a disjunction of goal facts
    #     # i.e. we can reach the goal if any of the goal facts is true

    #     # assume a disjunct set of goal facts for now.
    #     goal_p = sum(self.abstract_state.prob(f.svar, f.value) for f in goal_facts)
        
    #     # calculate disconfirm gain on how much it reduces the entropy of the goal formula
    #     # H_goal = -goal_p *math.log(goal_p, 2) - (1-goal_p) * math.log(1-goal_p,2)
    #     H_goal = self.entropy_of_goal(None, goal_facts)
    #     log.debug("P(goal) = %.3f, H(goal) = %.3f", goal_p, H_goal)


    #     # dis_score = float(confirm_score)/(2**(len(disconfirm)-1))
    #     disconfirm_actions = []
    #     for svar, val in disconfirm:
    #         dis_fact = state.Fact(svar, val)
    #         H = self.entropy_of_goal(dis_fact, goal_facts)
    #         log.debug("entropy of disconfirming %s: %.3f", dis_fact, H)
    #         if H_goal > 0:
    #             reward_factor = 1 - H/max(H, H_goal)
    #         else:
    #             reward_factor = 0
                
    #         # dH = 0
    #         # count = 0
    #         # for goalvar in set(g.svar.nonmodal() for g in goals):
    #         #     if goalvar in self.global_relevant:
    #         #         dH +=  self.calc_disconfirm_gain(dis_fact, goalvar)
    #         #         count += 1

    #         # if dH == 0:
    #         #     continue
    #         # dH = dH/count
                
    #         dis_erward = float(dt_conf.confirm_score - 20) * reward_factor #/(2**(max_level-level))
    #         # dis_reward = float(dt_conf.confirm_score) /(2**(max_level-levels[(svar,val)]))
    #         if isinstance(self.abstract_state[svar], pddl.TypedObject):
    #             continue # no use in disconfirming known facts
    #         p = self.abstract_state[svar][val]
    #         dis_penalty = -dis_reward * (1-p)/p

            
    #         term = pddl.Term(svar.function, svar.get_args())
    #         # p = self.abstract_state[svar][val]
    #         # if p < 0.001 or p > 0.999:
    #         #     continue # no use in disconfirming known facts

    #         # dis_penalty = -dis_score * (1-p)/p
            
            
    #         name = "disconfirm-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
    #         self.disconfirm_dict[name] = state.Fact(svar, val)
    #         a = pddl.Action(name, [], None, None, domain)
    #         b = pddl.builder.Builder(a)

    #         #conds = [b.cond('not', (dtpddl.committed, term))]
    #         #conds = [b.cond('not', ('done',))]
    #         # for gvar in goals:
    #         #     # don't allow this goal after a commit has been done
    #         #     gterm = pddl.Term(gvar.function, gvar.get_args())
    #         #     conds.append(b.cond('not', (dtpddl.committed, gterm)))
    #         # for ca in commit_actions:
    #         #     # don't allow commit actions after any disconfirm
    #         #     ca.precondition.parts.append(b.cond('not', (dtpddl.committed, term)))
            
    #         #a.precondition = pddl.Conjunction(conds, a)
    #         a.precondition = b.cond('not', ('done', ))
            
    #         #commit_effect = b.effect(dtpddl.committed, term)
    #         reward_effect = b('when', ('not', ('=', term, val)), ('assign', ('reward',), dis_reward))
    #         penalty_effect = b('when', ('=', term, val), ('assign', ('reward',), dis_penalty))
    #         done_effect = b.effect('done')
    #         a.effect = b.effect('and', reward_effect, penalty_effect, done_effect, clear_state_effect())
            
    #         disconfirm_actions.append(a)

    #     return commit_actions + disconfirm_actions
