from collections import defaultdict
import time, itertools
import config

import pddl
import pddl.dtpddl
from pddl import state
import plans

log = config.logger("planner")

def MAPLAction(action, task):
    elmts = action.split()
    action, args = elmts[0], elmts[1:]
    actions = dict((a.name,a) for a in task._mapldomain.actions)
    assert action in actions
    action_def = actions[action]
    args = [task._mapltask[a] for a in args[:len(action_def.args)]]
    return action_def, args

def getGoalDescription(goal, gnode, _state):
    gnode.action = plans.GoalAction(goal)
    
    read_vars = []
    universal_args = []
    _state.clear_axiom_cache()
    extstate, reasons, universalReasons = _state.get_extended_state(_state.get_relevant_vars(goal), getReasons=True)
        
    sat = extstate.is_satisfied(goal, read_vars, universal_args)
    assert sat
    
    read = set(read_vars)
    orig_read = set(read_vars)
    universal = set(a.type for a in universal_args)
    for v in read_vars:          
        if v in reasons:
            read.remove(v)
            read |= reasons[v]
        if v in universalReasons:
            universal |= set(a.type for a in universalReasons[v])

    log.trace("variables from goal:")
    log.trace("read: %s", " ".join(map(str, read)))

    gnode.preconds = set(state.Fact(var, _state[var]) for var in read)
    gnode.preconds_universal = universal
    gnode.explanations = reasons
    gnode.original_preconds = set(state.Fact(var, _state[var]) for var in orig_read)

    @pddl.visitors.collect
    def find_softgoals(elem, results):
        if isinstance(elem, pddl.conditions.PreferenceCondition):
            return [elem.cond]

    gnode.satisfied_softgoals = set()
    for sg in goal.visit(find_softgoals):
        if extstate.is_satisfied(sg):
            gnode.satisfied_softgoals.add(sg)
        
    return gnode

def get_conditional_effects(effects):
    def ceff_visitor(eff, results):
        results = sum(results, [])
        assert(not isinstance(eff, pddl.effects.UniversalEffect))
        if isinstance(eff, pddl.effects.SimpleEffect):
            return [(None, eff)]
        if isinstance(eff, pddl.effects.ConditionalEffect):
            new_res = []
            for cond, effs in results:
                new_res.append((pddl.Conjunction.join([cond, eff.condition]), effs))
            return new_res
        elif isinstance(eff, pddl.effects.ConjunctiveEffect):
            return results
        else:
            assert False, eff

    for eff in effects:
        for cond, ceffs in pddl.visitors.visit(eff, ceff_visitor, []):
            if cond is not None:
                yield cond, ceffs

def getRWDescription(action, args, _state, time):
    cond_keffs = pddl.translators.Translator.get_annotations(_state.problem.domain).get('has_commit_actions', False)
    pnode = plans.PlanNode(action, args, time, plans.ActionStatusEnum.EXECUTABLE)

    log.trace("get description for action (%s %s)", action.name, " ".join(a.name for a in action.args))

    action.instantiate(args, _state.problem)

    def process_conditions(st, read_vars, universal_args, reasons, universal_reasons):
        conds = set(read_vars)
        original_conds = set(state.Fact(var, st[var]) for var in read_vars)
        universal = set(a.type for a in universal_args)
        for v in set(read_vars):
            if v in reasons:
                conds.remove(v)
                conds |= reasons[v]
            if v in universal_reasons:
                universal |= set(a.type for a in universal_reasons[v])
        return conds, original_conds, universal

    #t0 = time.time()
    if action.replan:
        read_vars = []
        universal_args = []
        _state.clear_axiom_cache()
        extstate, reasons, universalReasons = _state.get_extended_state(_state.get_relevant_vars(action.replan), getReasons=True)
        sat = extstate.is_satisfied(action.replan, read_vars, universal_args)
        assert sat, "%s: %s" % (str(pnode), action.replan.pddl_str())

        pnode.replanconds, pnode.original_replan, pnode.replan_universal = process_conditions(extstate, read_vars, universal_args, reasons, universalReasons)
    #print "replan:", time.time()-t0
    #t0 = time.time()

    if action.precondition:
        read_vars = []
        universal_args = []
        rel = _state.get_relevant_vars(action.precondition)
        # print map(str, rel)
        _state.clear_axiom_cache()
        extstate, reasons, universalReasons = _state.get_extended_state(rel, getReasons=True)
        # print [(str(k), map(str,v)) for k,v in reasons.iteritems()]
        # print [(str(k), str(v)) for k,v in universalReasons.iteritems()]
        sat = extstate.is_satisfied(action.precondition, read_vars, universal_args)
        assert sat,  "%s: %s" % (str(pnode), action.precondition.pddl_str())

        pnode.preconds, pnode.original_preconds, pnode.preconds_universal = process_conditions(extstate, read_vars, universal_args, reasons, universalReasons)

    #print "read:", time.time()-t0

    #t0 = time.time()
    _state.read_svars.clear()
    pnode.effects = set()
    effects = []
    if action.effect:
        effects.append(action.effect)
    if isinstance(action, pddl.mapl.MAPLAction) and action.sensors:
        # print "action %s has sensors" % action.name
        if cond_keffs:
            effects.append(action.conditional_knowledge_effect())
            # print action.conditional_knowledge_effect().pddl_str()
        else:
            effects.append(action.knowledge_effect())
            # print action.conditional_knowledge_effect().pddl_str()
            
    if effects:
        rel = reduce(lambda x,y: x | y, (_state.get_relevant_vars(eff) for eff in effects), set())
        extstate, reasons, universalReasons = _state.get_extended_state(rel, getReasons=True)
        pnode.effects = set()
        for eff in effects:
            pnode.effects |= set(state.Fact(k,v) for k,v in extstate.get_effect_facts(eff, trace_vars=True).iteritems())

        eff_conds, original_eff_conds, universal_eff_conds = process_conditions(extstate, extstate.read_svars, [], reasons, universalReasons)
        pnode.preconds |= eff_conds
        pnode.original_preconds |= original_eff_conds
        pnode.preconds_universal |= universal_eff_conds

    pnode.ceffs = []

    for cond, ceff in get_conditional_effects(effects):
        read_vars = []
        universal_args = []
        rel = _state.get_relevant_vars(cond)
        _state.clear_axiom_cache()
        extstate, reasons, universalReasons = _state.get_extended_state(rel, getReasons=True)
        negcond = cond.negate()
        sat = extstate.is_satisfied(negcond, read_vars, universal_args)
        if sat:
            preconds, original_preconds, preconds_universal = process_conditions(extstate, read_vars, universal_args, reasons, universalReasons)
            preconds = set(state.Fact(var, _state[var]) for var in preconds)
            # for eff in ceffs:
            svar = state.StateVariable.from_literal(ceff)
            cfact = state.Fact(svar, _state[svar])
            pnode.ceffs.append((cfact, preconds, original_preconds, negcond, preconds_universal))
            
        
    #print "write:", time.time()-t0

    pterm = pddl.Term(pddl.dtpddl.probability, [])
    prob_terms = action.get_effects(pterm)
    if prob_terms:
        pnode.prob = _state.evaluate_term(prob_terms[0]).value

    cost_term = action.get_total_cost()
    if cost_term:
        pnode.cost = _state.evaluate_term(cost_term).value
        # if isinstance(cost_term, pddl.ConstantTerm):
        #     pnode.cost = cost_term.object.value
        # elif isinstance(cost_term, pddl.VariableTerm):
        #     pnode.cost = cost_term.get_instance().value
        # else:
        #     assert isinstance(cost_term, pddl.FunctionTerm)
        #     val = _state.evaluate_term(cost_term)
        #     pnode.cost = val.value
            
        
    #t0 = time.time()
    pnode.replanconds = set(state.Fact(var, _state[var]) for var in pnode.replanconds)
    pnode.preconds = set(state.Fact(var, _state[var]) for var in pnode.preconds)
    #print "stuff:", time.time()-t0

    log.trace("variables from %s:", action.name)
    log.trace("read: %s", " ".join(map(str, pnode.preconds)))
    log.trace("original read: %s", " ".join(map(str, pnode.original_preconds)))
    log.trace("replan: %s", " ".join(map(str, pnode.replanconds)))
    log.trace("original replan: %s", " ".join(map(str, pnode.original_replan)))
    log.trace("write: %s", " ".join(map(str, pnode.effects)))
    
    action.uninstantiate()
    return pnode


def make_po_plan(actions, task):
    #annotations = pddl.translators.Translator.get_annotations(task.mapltask)
    #soft_goals = annotations['soft_goals']
    #print annotations['soft_goals']
    
    t0 = time.time()
    plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())

    frontier = defaultdict(lambda: plan.init_node)
    
    readers = defaultdict(set)
    writers = defaultdict(set)

    ignored_soft_goals = set()
    
    state = task.get_state().copy()
    plan.init_node.effects = set(state.iterfacts())
    
    for starttime, action in actions:
        t1 = time.time()

        if action.startswith("ignore-preference-"):
            action = action[len("ignore-preference-"):]
            ignored_soft_goals.add(int(action))
            continue

        if action.startswith("fullfill-intermediate-"):
            continue

        action, args = MAPLAction(action, task)
        pnode = getRWDescription(action, args, state, starttime)
        plan.add_node(pnode)

        if not pnode.replanconds and not pnode.preconds:
            plan.add_link(plan.init_node, pnode, "started", pddl.TRUE)
            log.trace("%s depends artificially on init", pnode)
                         
        for svar, val in itertools.chain(pnode.replanconds, pnode.preconds):
            readers[svar].add(pnode)
            plan.add_link(frontier[svar], pnode, svar, val)
            log.trace("%s depends on %s", pnode, frontier[svar])
            
        for _, preconds, _, _, _ in pnode.ceffs:
            for svar, val in preconds:
                readers[svar].add(pnode)
                plan.add_link(frontier[svar], pnode, svar, val, conflict=False, ceff=True)
                log.trace("%s conditionally depends on %s", pnode, frontier[svar])

        for svar, val in itertools.chain(pnode.effects, (eff for eff, _, _, _, _ in pnode.ceffs)) :
            if svar.function in (pddl.builtin.total_cost, pddl.dtpddl.probability):
                continue
            if state[svar] != val:
                if svar in readers:
                    for node in readers[svar]:
                        if node != pnode and not node in plan.predecessors(pnode):
                            plan.add_link(node, pnode, svar, val, conflict=True)
                            log.trace("%s must preceed %s (read conflict)", node, pnode)
                    del readers[svar]

                if svar in writers:
                    for node in writers[svar]:
                        if node != pnode and not node in plan.predecessors(pnode):
                            plan.add_link(node, pnode, svar, val, conflict=True)
                            log.trace("%s must preceed %s (write conflict)", node, pnode)
                    del writers[svar]
                    
                state[svar] = val
            frontier[svar] = pnode
            writers[svar].add(pnode)

        log.trace("total time for action: %f", time.time()-t1)

    gnode = getGoalDescription(task.get_goal(), plan.goal_node, state)
    
    #plan.add_link(previous, plan.goal_node)

    for svar, val in gnode.preconds:
        plan.add_link(frontier[svar], plan.goal_node, svar, val)

#     for i,pnode in plan.topological_sort():
#         print i, str(pnode)
#         for to in plan.E[pnode]:
#             print "    ", to
    log.debug("total time for postprocessing: %f", time.time()-t0)

#    print "Ignored Soft Goals:"
#    for i in ignored_soft_goals:
#        print soft_goals[i]
    
    return plan
