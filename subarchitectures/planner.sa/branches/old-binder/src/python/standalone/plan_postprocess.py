from collections import defaultdict
import time, itertools
import config

from pddl import state
import plans

log = config.logger("planner")

def MAPLAction(action, task):
    elmts = action.split()
    action, args = elmts[0], elmts[1:]
    actions = dict((a.name,a) for a in task._mapltask.actions)
    assert action in actions
    action_def = actions[action]
    args = [task._mapltask[a] for a in args[:len(action_def.args)]]
    return action_def, args

def getGoalDescription(goal, _state):
    read_vars = []
    universal_args = []
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

    log.debug("variables from goal:")
    log.debug("read: %s", " ".join(map(str, read)))
    return set(state.Fact(var, _state[var]) for var in read), universal, set(state.Fact(var, _state[var]) for var in orig_read), reasons

def getRWDescription(action, args, _state, time):
    pnode = plans.PlanNode(action, args, time, plans.ActionStatusEnum.EXECUTABLE)
    
    action.instantiate(args)

    #t0 = time.time()
    if action.replan:
        read_vars = []
        universal_args = []
        extstate, reasons, universalReasons = _state.get_extended_state(_state.get_relevant_vars(action.replan), getReasons=True)
        sat = extstate.is_satisfied(action.replan, read_vars, universal_args), "%s: %s" % (str(pnode), action.replan.pddl_str())
        assert sat
        
        pnode.replanconds = set(read_vars)
        pnode.original_replan = set(state.Fact(var, extstate[var]) for var in read_vars)
        pnode.explanations.update(reasons)
        pnode.replan_universal = set(a.type for a in universal_args)
        for v in set(read_vars):
            if v in reasons:
                pnode.replanconds.remove(v)
                pnode.replanconds |= reasons[v]
            if v in universalReasons:
                pnode.replan_universal |= set(a.type for a in universalReasons[v])
    #print "replan:", time.time()-t0
    #t0 = time.time()

    if action.precondition:
        read_vars = []
        universal_args = []
        rel = _state.get_relevant_vars(action.precondition)
        extstate, reasons, universalReasons = _state.get_extended_state(rel, getReasons=True)
        sat = extstate.is_satisfied(action.precondition, read_vars, universal_args),  "%s: %s" % (str(pnode), action.precondition.pddl_str())
        assert sat
        pnode.preconds = set(read_vars)
        pnode.original_preconds = set(state.Fact(var, extstate[var]) for var in read_vars)
        pnode.explanations.update(reasons)
        pnode.preconds_universal = set(a.type for a in universal_args)
        for v in set(read_vars):
            if v in reasons:
                pnode.preconds.remove(v)
                pnode.preconds |= reasons[v]
            if v in universalReasons:
                pnode.preconds_universal |= set(a.type for a in universalReasons[v])

    #print "read:", time.time()-t0

    #t0 = time.time()
    _state.read_svars.clear()
    pnode.effects = set()
    if action.effect:
        pnode.effects = set(state.Fact(k,v) for k,v in _state.get_effect_facts(action.effect, trace_vars=True).iteritems())
    if action.sensors:
        pnode.effects |= set(state.Fact(k,v) for k,v in _state.get_effect_facts(action.knowledge_effect() , trace_vars=True).iteritems())

    pnode.preconds |= _state.read_svars
    pnode.original_preconds |= set(state.Fact(var, extstate[var]) for var in _state.read_svars)
    #print "write:", time.time()-t0
        
    #t0 = time.time()
    pnode.replanconds = set(state.Fact(var, _state[var]) for var in pnode.replanconds)
    pnode.preconds = set(state.Fact(var, _state[var]) for var in pnode.preconds)
    #print "stuff:", time.time()-t0

    log.debug("variables from %s:", action.name)
    log.debug("read: %s", " ".join(map(str, pnode.preconds)))
    log.debug("original read: %s", " ".join(map(str, pnode.original_preconds)))
    log.debug("replan: %s", " ".join(map(str, pnode.replanconds)))
    log.debug("original replan: %s", " ".join(map(str, pnode.original_replan)))
    log.debug("write: %s", " ".join(map(str, pnode.effects)))
    
    action.uninstantiate()
    return pnode


def make_po_plan(actions, task):
    t0 = time.time()
    plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())

    relevant_init_vars = set()
    frontier = defaultdict(lambda: plan.init_node)
    
    readers = defaultdict(set)
    writers = defaultdict(set)
    
    state = task.get_state().copy()
    previous = plan.init_node
    
    for starttime, action in actions:
        t1 = time.time()
        action, args = MAPLAction(action, task)
        pnode = getRWDescription(action, args, state, starttime)
        plan.add_node(pnode)
        #linear plan for now
        #plan.add_link(previous, pnode)
                         
        for svar, val in itertools.chain(pnode.replanconds, pnode.preconds):
            if svar not in frontier:
                relevant_init_vars.add(svar)
            readers[svar].add(pnode)
            plan.add_link(frontier[svar], pnode, svar, val)
            log.debug("%s depends on %s", pnode, frontier[svar])

        for svar, val in pnode.effects:
            if state[svar] != val:
                if svar in readers:
                    for node in readers[svar]:
                        if node != pnode and not node in plan.predecessors(pnode):
                            plan.add_link(node, pnode, svar, val, conflict=True)
                            log.debug("%s must preceed %s (read conflict)", node, pnode)
                    del readers[svar]

                if svar in writers:
                    for node in writers[svar]:
                        if node != pnode and not node in plan.predecessors(pnode):
                            plan.add_link(node, pnode, svar, val, conflict=True)
                            log.debug("%s must preceed %s (write conflict)", node, pnode)
                    del writers[svar]
                    
                state[svar] = val
            frontier[svar] = pnode
            writers[svar].add(pnode)

        previous = pnode
        log.debug("total time for action: %f", time.time()-t1)

    read, universal, orig_read, explanations = getGoalDescription(task.get_goal(), state)
    plan.goal_node.preconds = read
    plan.goal_node.preconds_universal = universal
    plan.goal_node.action = plans.GoalAction(task.get_goal())
    plan.goal_node.explanations = explanations
    plan.goal_node.original_preconds = orig_read
    
    #plan.add_link(previous, plan.goal_node)

    for svar, val in read:
        if svar not in frontier:
            relevant_init_vars.add(svar)
        plan.add_link(frontier[svar], plan.goal_node, svar, val)

#     for i,pnode in plan.topological_sort():
#         print i, str(pnode)
#         for to in plan.E[pnode]:
#             print "    ", to
    log.debug("total time for postprocessing: %f", time.time()-t0)
    
    return plan
