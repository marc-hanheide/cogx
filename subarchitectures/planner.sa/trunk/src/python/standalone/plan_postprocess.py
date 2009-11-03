from collections import defaultdict
import time, itertools

import mapl_new as mapl
import state_new as state
import plans

def MAPLAction(action, task):
    elmts = action.split()
    action, args = elmts[0], elmts[1:]
    actions = dict((a.name,a) for a in itertools.chain(task._mapltask.actions, task._mapltask.sensors))
    assert action in actions
    action_def = actions[action]
    args = [task._mapltask[a] for a in args]
    return action_def, args

def getGoalDescription(goal, _state):
    read_vars = []
    universal_args = []
    extstate, reasons, universalReasons = _state.getExtendedState(_state.getRelevantVars(goal), getReasons=True)
    extstate.isSatisfied(goal, read_vars, universal_args)
    read = set(read_vars)
    universal = set(a.type for a in universal_args)
    for v in read_vars:          
        if v in reasons:
            read.remove(v)
            read |= reasons[v]
        if v in universalReasons:
            universal |= set(a.type for a in universalReasons[v])

    return set(state.Fact(var, _state[var]) for var in read), universal

def getRWDescription(action, args, _state):
    action.instantiate(args)
    replan=set()
    replan_universal=set()
    read=set()
    read_universal=set()
    write=set()

    #t0 = time.time()
    if action.replan:
        read_vars = []
        universal_args = []
        extstate, reasons, universalReasons = _state.getExtendedState(_state.getRelevantVars(action.replan), getReasons=True)
        extstate.isSatisfied(action.replan, read_vars, universal_args)
        replan = set(read_vars)
        replan_universal = set(a.type for a in universal_args)
        for v in read_vars:
            if v in reasons:
                replan.remove(v)
                replan |= reasons[v]
            if v in universalReasons:
                replan_universal |= set(a.type for a in universalReasons[v])
    #print "replan:", time.time()-t0
    #t0 = time.time()

    if action.precondition:
        read_vars = []
        universal_args = []
        rel = _state.getRelevantVars(action.precondition)
        extstate, reasons, universalReasons = _state.getExtendedState(rel, getReasons=True)
        extstate.isSatisfied(action.precondition, read_vars, universal_args)
        read = set(read_vars)
        read_universal = set(a.type for a in universal_args)
        for v in read_vars:
            if v in reasons:
                read.remove(v)
                read |= reasons[v]
            if v in universalReasons:
                read_universal |= set(a.type for a in universalReasons[v])

    #print "read:", time.time()-t0

    #t0 = time.time()
    if isinstance(action, mapl.sensors.Sensor):
        effects = [action.knowledge_effect()]
    else:
        effects = action.effects
    for eff in effects:
        write |= _state.getEffectFacts(eff)
    #print "write:", time.time()-t0
        
    #t0 = time.time()
    replan = set(state.Fact(var, _state[var]) for var in replan)
    read = set(state.Fact(var, _state[var]) for var in read)
    #print "stuff:", time.time()-t0

    action.uninstantiate()
    return replan, replan_universal, read, read_universal, write


def make_po_plan(actions, task):
    t0 = time.time()
    plan = plans.MAPLPlan(init_state=task.get_state(), goal_condition=task.get_goal())

    relevant_init_vars = set()
    frontier = defaultdict(lambda: plan.init_node)
    
    readers = defaultdict(set)
    
    state = task.get_state().copy()
    previous = plan.init_node
    
    for starttime, action in actions:
        t1 = time.time()
        action, args = MAPLAction(action, task)
        pnode = plans.PlanNode(action, args, starttime, plans.ActionStatusEnum.EXECUTABLE)
        plan.add_node(pnode)
        #linear plan for now
        plan.add_link(previous, pnode)
                 
        replan, replan_universal, read, read_universal, write = getRWDescription(action, args, state)

        pnode.preconds = read
        pnode.replanconds = replan
        pnode.effects = write

        pnode.replan_universal = replan_universal
        pnode.preconds_universal = read_universal

        for svar, val in itertools.chain(replan, read):
            if svar not in frontier:
                relevant_init_vars.add(svar)
            readers[svar].add(pnode)
            #plan.add_link(frontier[svar], pnode)

        for svar, val in write:
            if state[svar] != val:
                if svar in readers:
                    #for node in readers[svar]:
                    #    if node != pnode:
                    #        plan.add_link(node, pnode)
                    del readers[svar]
                
                frontier[svar] = pnode
                state[svar] = val

        previous = pnode
        #print "total time for action:", time.time()-t1

    read, universal = getGoalDescription(task.get_goal(), state)
    plan.goal_node.preconds = read
    plan.goal_node.preconds_universal = universal
    plan.goal_node.action = plans.GoalAction(task.get_goal())
    
    plan.add_link(previous, plan.goal_node)

    for svar, val in read:
        if svar not in frontier:
            relevant_init_vars.add(svar)
        #plan.add_link(frontier[svar], plan.goal_node)

#     for i,pnode in plan.topological_sort():
#         print i, str(pnode)
#         for to in plan.E[pnode]:
#             print "    ", to
#    print "total time for postprocessing:", time.time()-t0
    
    return plan
