from collections import defaultdict
import itertools
import config

import pddl
from pddl import state, visitors, mapl
import plans

log = config.logger("assertions")

def get_observable_functions(sensors):
    result=[]
    for s in sum((sensor.sensors for sensor in sensors), []):
        pred_types = [a.get_type() for a in s.get_term().args]
        if s.is_boolean():
            pred_types.append(s.get_value().get_type())
        else:
            pred_types.append(s.get_term().get_type())

        #Unify predicates with same/compatible types
        exists = False
        remove = []
        for function, types in result:
            if function != s.get_term().function:
                continue

            is_supertype = False
            is_subtype = False
            compatible = True
            for newtype, oldtype in zip(pred_types, types):
                if newtype == arg.get_type():
                    continue
                elif newtype.is_subtype_of(oldtype):
                    is_subtype = True
                elif newtype.is_supertype_of(oldtype):
                    is_supertype = True
                else:
                    compatible = False
                    break

            #some parameters are subtypes, some are supertypes of the existing predicate
            #Those can't be unified
            if is_supertype and is_subtype:
                compatible = False
            #Remove existing predicate if current one is a strict superype
            if compatible and is_supertype:
                remove.append((function, types))
            elif is_equal:
                exists = True
                break

        for rem in remove:
            result.remove(rem)

        if not exists:
            result.append((s.get_term().function, pred_types))
        
    observable = defaultdict(list)
    for func, types in result:
        observable[func].append(types)
    return observable

def get_nonstatic_functions(domain):
    @visitors.collect
    def effectVisitor(eff, results):
        if isinstance(eff, pddl.SimpleEffect):
            if eff.predicate in pddl.assignment_ops:
                return eff.args[0].function
            elif eff.predicate in mapl.modal_predicates:
                return None
            else:
                return eff.predicate
                
    result = set()
    for a in domain.actions:
        result |= set(a.effect.visit(effectVisitor))
    return result

def get_static_functions(domain):
    nonstatic = get_nonstatic_functions(domain)
    result = set()
    for func in itertools.chain(domain.predicates, domain.functions):
        if func not in nonstatic:
            result.add(func)

    return result
        
def is_observable(observables, term, value=None):
    if term.function not in observables:
        return False

    ttypes = [a.get_type() for a  in term.args]
    if value:
        ttypes.append(value.get_type())
    else:
        ttypes.append(term.function.type)

    for types in observables[term.function]:
        if all(map(lambda t1,t2: t1.equal_or_subtype_of(t2) or t2.equal_or_subtype_of(t1), ttypes, types)):
            return True
    return False


tr = mapl.MAPLObjectFluentNormalizer()

def to_assertion(action, domain):
    if not action.precondition:
        return None
    
    observable = get_observable_functions(domain.actions)
    action = tr.translate(action, domain=domain)
    #print action.precondition.pddl_str()
    agent = action.agents[0]

    new_indomain = set()

    def assertionVisitor(cond, results=[]):
        if cond.__class__ == pddl.LiteralCondition:
            if cond.predicate == mapl.knowledge:
                return None, cond
            if cond.predicate == pddl.equals and all(not isinstance(t, pddl.FunctionTerm) for t in cond.args):
                return cond, None
            if cond.predicate != pddl.equals or \
                    not is_observable(observable, cond.args[0], cond.args[1]) :
                return cond, None

            k_cond = pddl.LiteralCondition(mapl.knowledge, [pddl.VariableTerm(agent), cond.args[0]])
            id_cond = pddl.LiteralCondition(mapl.indomain, cond.args[:], negated = cond.negated)
            #print cond.pddl_str()

            return id_cond, k_cond
        elif isinstance(cond, pddl.conditions.JunctionCondition):
            condparts = []
            replanparts = []
            newcond = newreplan = None
            for pre, replan in results:
                if pre: condparts.append(pre)
                if replan: replanparts.append(replan)
            if condparts:
                newcond = cond.__class__(condparts)
            if replanparts:
                newreplan = cond.__class__(replanparts)
                    
            return newcond, newreplan
        elif cond.__class__ == pddl.conditions.QuantifiedCondition:
            newcond = newreplan = None
            if results[0][0]:
                newcond = cond.__class__(self.args, condparts)
            if results[0][1]:
                newreplan = cond.__class__(self.args, replanparts)
            return newcond, newreplan
        else:
            assert False

    condition, replan = action.precondition.visit(assertionVisitor)

    if not replan:
        return None

    # eff = action.effect
    # if action.sensors:
    #     keff = action.knowledge_effect()
    #     if eff:
    #         keff.parts.insert(0, eff)
    #     eff = keff

    assertion = mapl.MAPLAction("assertion_"+action.name, action.agents, action.maplargs, action.vars, condition, replan, action.effect, action.sensors, domain)
    #cost_eff = pddl.SimpleEffect(pddl.builtin.increase, [pddl.Term(pddl.builtin.total_cost, []), pddl.Term(1)])
    #if isinstance(assertion.effect, pddl.ConjunctiveEffect):
    #    assertion.effect.parts.append(cost_eff)
    #else:
    #    assertion.effect = pddl.ConjunctiveEffect([assertion.effect, cost_eff])
    
    assertion = assertion.copy()
    return assertion

def cluster_predecessors(cluster, plan):
    result = set()
    for node in cluster:
        result |= set(pred for pred in plan.predecessors_iter(node) if not isinstance(pred, plans.DummyAction))
    return result - cluster

def cluster_successors(cluster, plan):
    result = set()
    for node in cluster:
        result |= set(pred for pred in plan.successors_iter(node) if not isinstance(pred, plans.DummyAction))
    return result - cluster

def get_relevant_effects(cluster, plan):
    successors = cluster_successors(cluster, plan)
    predecessors = cluster_predecessors(cluster, plan)

    read_later = set()
    written = set()
    for pnode in cluster:
        written |= set(svar for svar,val in pnode.effects)
    for pnode in successors:
        read_later |= set(svar for svar,val in itertools.chain(pnode.preconds, pnode.replanconds))

    result = written & read_later 
    for pnode in predecessors:
        readonly = set(svar for svar,val in itertools.chain(pnode.preconds, pnode.replanconds)) - set(svar for svar,val in pnode.effects)
        threat_vars = readonly & written
        result |= threat_vars
    return result


def make_clusters(plan, domain):
    log = config.logger("assertions.clustering")
    results = set()
    initial_nodes = []
    static_functions = get_static_functions(domain)

    results |= find_clusters_from_goal(plan.goal_node, plan, static_functions, domain)

    # for node in plan.nodes_iter():
    #     for pred in plan.predecessors_iter(node):
    #         if isinstance(pred.action, pddl.sensors.Sensor):
    #             if any(e['type'] == 'depends' for e in plan[pred][node].itervalues()):
    #                 initial_nodes.append(node)
    #                 break

            
    # for node in initial_nodes:
    #     results |= add_to_cluster(set([node]), plan, static_functions, domain)
        

    log.info("Number of resulting clusters: %d", len(results))
    return results


def find_clusters_from_goal(goal_node, plan, static, domain):
    init = plan.predecessors(goal_node, 'depends')
    clusters = set()
    for start_node in init:
        cluster = find_subcluster(start_node, plan, static, domain)
        # open = set([start_node])
        # while open:
        #     pnode = open.pop()
        #     for pred in plan.predecessors_iter(pnode, 'depends'):
        #         if not isinstance(pred.action, pddl.sensors.Sensor):
        #             cluster.add(pred)
        #             open.add(pred)
                
        if len(cluster) > 1:
            clusters.add(frozenset(cluster))
    return clusters

def find_subcluster(pnode, plan, static, domain):
    subcluster = set()
    for pred in plan.predecessors_iter(pnode, 'depends'):
        if pred.action == plan.init_node:
            continue
        if isinstance(pred.action, pddl.sensors.Sensor):
            subcluster.add(pnode)
        subcluster |= find_subcluster(pred, plan, static, domain)
        
    if subcluster:
        subcluster.add(pnode)
    return subcluster
    

def add_to_cluster(cluster, plan, static, domain):
    log = config.logger("assertions.clustering")
    
    def check_consistency(cluster, node):
        if any(plan.pred_closure(n, 'depends') & cluster for n in plan.pred_closure(node, 'depends') - cluster):
            return False
        if any(plan.succ_closure(n, 'depends') & cluster for n in plan.succ_closure(node, 'depends') - cluster):
            return False
        return True

    def get_svars(cluster, only_nonstatic=False, filter_func=None):
        if only_nonstatic:
            filter_func = lambda svar: svar.function not in static and svar.modality is None
        read = set()
        write = set()
        for pnode in plan.topological_sort():
            if pnode not in cluster:
                continue
            read |= set(svar for svar,val in itertools.chain(pnode.preconds, pnode.replanconds) if filter_func(svar)) - write
            write |= set(svar for svar,val in pnode.effects if filter_func(svar))
            
        return read, write, read|write

    def get_sensor_pres(cluster):
        read, _, _ = get_svars(cluster, filter_func=lambda svar: svar.modality == mapl.direct_knowledge)
        return read
    
    candidates = set()
    for a in cluster:
        candidates |= set(plan.predecessors(a, 'depends') + plan.successors(a, 'depends')) - cluster
    
    candidates.discard(plan.init_node)
    candidates.discard(plan.goal_node)

    add_always_candidates = set()
    branch_candidates = set()

    nonstatic_pre, nonstatic_eff, nonstatic_svars = get_svars(cluster, only_nonstatic=True)
    sensor_pre = get_sensor_pres(cluster)
    relevant_effs = get_relevant_effects(cluster, plan)

    log.debug("cluster: %s", map(str, cluster))
    
    for a in candidates:
        log.debug("considering %s ...", a)
        if not check_consistency(cluster, a):
            log.debug("inconsistent")
            continue

        nonstatic_pre2, nonstatic_eff2, nonstatic_svars2 = get_svars(cluster | set([a]), only_nonstatic=True)
        sensor_pre2 = get_sensor_pres(cluster | set([a]))
        relevant_effs2 = get_relevant_effects(cluster | set([a]), plan)

        log.debug("sensor preconditions: %d, %d", len(sensor_pre), len(sensor_pre2))
        sensors_added = len(sensor_pre2 - sensor_pre)
        sensors_removed = len(sensor_pre - sensor_pre2)

        nonstatic_pre_added = len(nonstatic_pre2 - nonstatic_pre - sensor_pre2)
        nonstatic_pre_removed = len(nonstatic_pre - nonstatic_pre2 - sensor_pre)

#        relevant_eff_added = len(relevant_effs2 - relevant_effs)
#        relevant_eff_removed = len(relevant_effs - relevant_effs2)

#        svars_added = len(relevant_effs2 | nonstatic_pre2 - (relevant_effs | nonstatic_pre))
#        svars_removed = len(relevant_effs | nonstatic_pre2 - (relevant_effs2 | nonstatic_pre2))

        #only non-negative traits:
#        if sensors_removed == 0 and nonstatic_pre_added == 0 and relevant_eff_added == 0:
        if sensors_removed == 0 and nonstatic_pre_added == 0:
            add_always_candidates.add(a)
            log.debug("added")
#        elif sensors_added - sensors_removed >= 0 and nonstatic_pre_removed - nonstatic_pre_added > 0:
        elif sensors_added - sensors_removed + nonstatic_pre_removed - nonstatic_pre_added > 0:
#        elif sensors_added - sensors_removed + svars_removed - svars_added > 0:
            branch_candidates.add(a)
            log.debug("branched")
        else:
            log.debug("not added")

        

    results = set()

    cluster |= add_always_candidates;
        
    if add_always_candidates:
        results |= add_to_cluster(cluster.copy(), plan, static, domain)
    elif len(cluster) > 1:
        results.add(frozenset(cluster))
        
    for c in branch_candidates:
        branch = cluster.copy()
        branch.add(c)
        results |= add_to_cluster(branch, plan, static, domain)

    return results
