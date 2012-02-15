from itertools import chain
from standalone import pddl, plans, config
from standalone.pddl import types
from standalone.pddl.builtin import t_number 

# some globals that will be used all over the module once set
expl_domain = None
se_condition = None
w = None

ignored_functions = set()

def fact_filter(fact):
    if fact.svar.function in ignored_functions:
        return False
    if fact.svar.function.name in ("enabled", "phase"):
        return False
    return fact.svar.get_type() != pddl.t_number


SWITCH_OP1 = """
  (:action _switch_phase_simulate_execution
   :precondition (= (phase) apply_rules)
   :effect (and (assign (phase) simulate_execution))
  )
"""

SWITCH_OP2 = """
  (:action _switch_phase_apply_rules
   :precondition (= (phase) make_alternative_assumptions)
   :effect (and (assign (phase) apply_rules))
  )
"""


compatible_axiom = """
(:derived (compatible ?svar - (function object)  ?val - (typeof ?svar))
          (or (poss ?svar ?val)
              (not (exists (?val2 - (typeof ?svar)) (poss ?svar ?val2))))
)
"""
p = pddl.Parameter("?f", types.FunctionType(pddl.t_object))
compatible = pddl.Predicate("compatible", [p, pddl.Parameter("?v", pddl.types.ProxyType(p)), ])
state_rules = []

def str2cond(fstr, scope):
    return pddl.parser.Parser.parse_as([fstr], pddl.conditions.Condition, scope)

def str2eff(fstr, scope=None):
    return pddl.parser.Parser.parse_as([fstr], pddl.effects.Effect, scope)

def add_explanation_rules(expl_rules_fn):
    global expl_domain
    expl_domain.alternatives = {}
    p = pddl.parser.Parser.parse_file(expl_rules_fn)
    it = iter(p.root)
    it.get("rules")
    j = iter(it.get(list, "(domain 'domain identifier')"))
    j.get("domain")
    domname = j.get(None, "domain identifier").token.string
    # check that domain name matches the planning domain
    # tbd
    for elem in it:
        j = iter(elem)
        type = j.get("terminal").token
        if type == ":action":
            a = pddl.mapl.MAPLAction.parse(j.reset(), expl_domain)
            alist = expl_domain.actions
        elif type == ":init-rule":
            a = pddl.mapl.InitRule.parse(j.reset(), expl_domain) 
            alist = expl_domain.init_rules
        else:
            raise ParseError(type, "Unknown section identifier: '%s'." % type.string)
        try:
            expl_domain.domain_orig.get_action(a.name)
            expl_domain.alternatives[a.name] = a
        except KeyError:
            if a.name.startswith("state_rule"):
                state_rules.append(a)
            else:
                alist.append(a)
                if isinstance(a, pddl.Action):
                    a.extend_precondition(str2cond("(= (phase) apply_rules)", expl_domain))

def build_operator_for_ground_action(i, action, args):
    action_t = expl_domain.types["action"]
    action_id = "action_" + str(i).zfill(3)
    expl_domain.add_constant(types.TypedObject(action_id, action_t))
    name = "_%s_%s" % (action_id, action.name)
    new_op = pddl.Action(name, [], None, None, None, None)
    action.instantiate(args, expl_domain)
    if action.precondition:
        new_op.precondition = action.precondition.copy(copy_instance=True)
    #new_op.precondition = pddl.Conjunction([]) # TEST
    if action.effect:
        new_op.effect = action.effect.copy(copy_instance=True)
    enabled_cond = str2cond("(= (enabled) %s)" % action_id, expl_domain)
    for cond in [se_condition, enabled_cond]:
        new_op.extend_precondition(cond)
    enabled_eff = str2eff("(assign (enabled) %s)" % action_id, expl_domain)
    new_op.set_total_cost(pddl.Term(1))
    return [new_op], enabled_eff

def possible_cond_effect(fact):
    @pddl.visitors.collect
    def get_ceffs(elem, results):
        if isinstance(elem, pddl.effects.ConditionalEffect):
            return elem.effect.visit(pddl.visitors.collect_effects)
    for a in expl_domain.actions:
        for eff in pddl.visitors.visit(a.effect, get_ceffs, []):
            if fact.svar.match_literal(eff) is not None:
                return True
    return False

def possible_commit_effect(fact):
    if fact.svar.modality is not None or isinstance(fact.svar.function, pddl.Predicate):
        return False
    poss_var = fact.svar.as_modality(pddl.mapl.commit, [fact.value])
    for a in expl_domain.actions:
        if "new_facts" not in a.name:
            for eff in pddl.visitors.visit(a.effect, pddl.visitors.collect_effects, []):
                if poss_var.match_literal(eff) is not None:
                    return True
    return False

def state_rule_conditions(fact):
    pre = None
    for r in state_rules:
        mapping = fact.match_literal(r.effect)
        if mapping is not None:
            r.instantiate(mapping, expl_domain)
            pre = r.precondition.copy(copy_instance=True)
            r.uninstantiate()
            break
    return pddl.visitors.visit(pre, pddl.visitors.collect_conditions, [])

def build_operator_for_new_facts(i, node):
    compatibility_conds = set(pddl.state.Fact(f.svar.as_modality(compatible, [f.value]), pddl.TRUE) for f in node.preconds if possible_commit_effect(f))
    node.preconds = set(f for f in node.preconds if possible_cond_effect(f))

    rule_conditions = sum((state_rule_conditions(f) for f in node.effects if fact_filter(f)), [])
    
    action_t = expl_domain.types["action"]
    action_id = "action_" + str(i).zfill(3)
    expl_domain.add_constant(types.TypedObject(action_id, action_t))
    name = "_%s_%s" % (action_id, node.action.name)
    new_op = pddl.Action(name, [], None, None, expl_domain, None)
    new_op.precondition = pddl.Conjunction(rule_conditions + [f.to_condition() for f in chain(node.preconds, compatibility_conds) if fact_filter(f)], new_op)
    new_op.effect = pddl.ConjunctiveEffect([f.to_effect() for f in node.effects if f not in node.preconds and fact_filter(f)], new_op)
    add_ops = []
    for i, pre in enumerate(node.preconds):
        name = "forfeit_%s_%s-%d" % (action_id, node.action.name, i)
        new_op2 = pddl.Action(name, [], None, None, expl_domain, None)
        new_op2.effect = pre.to_effect()
        # new_op2.precondition = pddl.Conjunction([f.to_condition() for f in node.preconds if fact_filter(f)], new_op2)
        new_op2.set_total_cost(pddl.Term(500))
        add_ops.append(new_op2)
    
    enabled_cond = str2cond("(= (enabled) %s)" % action_id, expl_domain)
    for cond in [se_condition, enabled_cond]:
        for op in chain([new_op], add_ops):
            op.extend_precondition(cond)
    enabled_eff = str2eff("(assign (enabled) %s)" % action_id, expl_domain)

    return [new_op], add_ops, enabled_eff
    

def build_explanation_domain(last_plan, problem, expl_rules_fn):
    global se_condition, expl_domain, commitments
    domain_orig = problem.domain
    expl_domain = domain_orig.copy_skeleton()
    expl_domain.domain_orig = domain_orig
    expl_domain.name += "-explanations"
    expl_domain.axioms = [a.copy(newdomain=expl_domain) for a in domain_orig.axioms]
    # print [a.predicate.name for a in domain_orig.axioms]

    dtypes = expl_domain.types
    # add additional types
    for t in ["action", "phase"]:
        dtypes[t] = pddl.Type(t, [pddl.builtin.t_object])

    # add additional functions
    action_t = dtypes["action"]
    phase_t = dtypes["phase"]
    enabled_f = pddl.predicates.Function("enabled", [], action_t)
    phase_f = pddl.predicates.Function("phase", [], phase_t)
    for f in [enabled_f, phase_f]:
        expl_domain.functions.add(f)
    expl_domain.predicates.add(compatible)
    expl_domain.axioms.append(pddl.parser.Parser.parse_as(compatible_axiom.splitlines(), pddl.axioms.Axiom, expl_domain))

    # add objects from problem to domain as constants
    for obj in problem.objects:
        expl_domain.add_constant(obj)
    for phase in ["make_alternative_assumptions", "apply_rules", "simulate_execution", "achieve_goal"]:
        expl_domain.add_constant(types.TypedObject(phase, phase_t))
    aa_condition = str2cond("(= (phase) make_alternative_assumptions)", expl_domain)
    ar_condition = str2cond("(= (phase) apply_rules)", expl_domain)
    se_condition = str2cond("(= (phase) simulate_execution)", expl_domain)

    for t in expl_domain.types.itervalues():
        if t in (pddl.t_object, pddl.t_boolean, pddl.t_number) + tuple(pddl.mapl.mapl_types):
            continue
        unknown_obj = pddl.TypedObject("unknown-%s" % t.name, t)
        expl_domain.add_constant(unknown_obj)

    # add rules for generating alternative initial states
    add_explanation_rules(expl_rules_fn)

    commitments = set()
   # extract actions from old plan
    last_actions = []
    i = 0
    for n in last_plan.topological_sort():
        print "***",n, n.status
        # iterate through plan, create action constants to enfore simulated re-execution during monitoring
        a = n.action
        if isinstance(a, plans.DummyAction) and not a.name.startswith("new_facts"):
            continue
        if n.status == plans.ActionStatusEnum.EXECUTABLE:
            continue
        if n.is_virtual():
            for succ, svar, val, type in last_plan.outgoing_links(n):
                if svar.modality == pddl.mapl.commit and not succ.is_virtual():
                    # print "link:", n, succ, svar, val
                    commitments.add(pddl.state.Fact(svar.as_modality(pddl.mapl.committed), pddl.TRUE))
            a = a.copy()
            a.extend_precondition(ar_condition)
            #print "virtual action:", w.write_action(a)  
            expl_domain.add_action(a)
            continue
        try:
            a = expl_domain.alternatives[a.name]
        except KeyError:
            pass
        if a.name.startswith("new_facts"):
            a2, add_ops, enabled_last = build_operator_for_new_facts(i, n)
            print "   ",  [a.name for a in a2]
        else:
            a2, enabled_last = build_operator_for_ground_action(i, a, n.full_args)
            print "   ",  [a.name for a in a2]
            add_ops = []
        for op in chain(a2, add_ops):
            expl_domain.add_action(op)
            
        for op in last_actions:
            op.extend_effect(enabled_last)
            
        last_actions = a2
        # if n.status == plans.ActionStatusEnum.FAILED:
        #     # once we've reached the failure, stop enforced execution simulation
        #     break
        i += 1
    for op in last_actions:
        op.extend_effect(str2eff("(assign (phase) achieve_goal)", expl_domain))
    # add action to switch phases
    for swop in [SWITCH_OP1, SWITCH_OP2]:
        switch_action = pddl.parser.Parser.parse_as(swop.splitlines(), pddl.Action, expl_domain)
        expl_domain.add_action(switch_action)


def build_explanation_problem(problem, last_plan, init_state, observed_state):
    assert expl_domain is not None
    p = problem.copy(newdomain=expl_domain)
    p.objects = set()

    p.init = [f.to_init() for f in init_state.iterfacts()]
    p.init.append(pddl.Builder(p).init("=", ("phase",), "apply_rules"))
    p.init.append(pddl.Builder(p).init("=", ("enabled",), "action_000"))

    relevant = set()
    for n in last_plan:
        if n.status != plans.ActionStatusEnum.EXECUTABLE and not n.is_virtual() and not n.action.name.startswith("new_facts") and not isinstance(n, plans.DummyNode):
            relevant |= n.effects

    # gfacts = [f.to_condition() for f in observed_state.iterfacts() if not f.value.is_instance_of(t_number)]
    # for f in sorted(observed_state.iterfacts(), key=str):
    #     print ":", f
    def filter_fn(fact):
        if not fact_filter(fact):
            return False
        return fact.value != pddl.FALSE and fact.svar.get_type() == pddl.t_boolean
    # extstate = observed_state.get_extended_state(set(f.svar for f in relevant)) #evaluate axioms
    # gfacts += [f.to_condition().negate() for f in relevant if f.svar not in extstate and filter_fn(f)]
    # for f in sorted(relevant,key=str):
    #     print f, observed_state[f.svar]
    gfacts = [f.to_condition() for f in commitments]
    goal = pddl.Conjunction(gfacts)
    #goal = pddl.Conjunction([])  # TEST
    if not isinstance(goal, pddl.Conjunction):
        goal = pddl.Conjunction([goal])
    goal.parts.append(str2cond("(= (phase) achieve_goal)", expl_domain))
    p.goal = goal
    return p
    

def handle_failure(last_plan, problem, init_state, observed_state, expl_rules_fn, cp_task, component):
    global w
    w = pddl.mapl.MAPLWriter()

    if "started" in problem.domain.predicates:
        ignored_functions.add(problem.domain.predicates["started"][0])

    sorted_plan = last_plan.topological_sort()

    build_explanation_domain(last_plan, problem, expl_rules_fn)
    # print "\n".join(w.write_domain(expl_domain))

    expl_problem = build_explanation_problem(problem, sorted_plan, init_state, observed_state)
    # print "\n".join(w.write_problem(expl_problem))
    
    cp_task.mapltask = expl_problem
    cp_task.set_state(pddl.state.State.from_problem(expl_problem))
    cp_task.set_plan(None)
    cp_task.mark_changed()
    print "planning"
    cp_task.replan()
    plan = cp_task.get_plan()
    print plan

    if plan is not None:
        result = postprocess_explanations(plan, last_plan)
        component.verbalise("I have now found a possible explanation for the failure.")
    else:
        component.verbalise("I'm sorry. I can't explain what went wrong.")
        result = []
        print "No explanations found"

    print "done"
    return result


def get_causal_relations(node):
    node.action.instantiate(node.full_args)
    @pddl.visitors.collect
    def get_ceffs(elem, results):
        if isinstance(elem, pddl.effects.ConditionalEffect):
            return elem
        
    for ceff in pddl.visitors.visit(node.action.effect, get_ceffs, []):
        conds = ceff.condition.visit(pddl.visitors.collect_literals)
        effs = ceff.effect.visit(pddl.visitors.collect_literals)
        c_facts = [pddl.state.Fact.from_literal(l) for l in conds]
        e_facts = [pddl.state.Fact.from_literal(l) for l in effs]
        yield c_facts, e_facts
    node.action.uninstantiate()
    
      
def postprocess_explanations(expl_plan, exec_plan):
    real_exec_actions = [n for n in exec_plan.topological_sort() if not n.is_virtual() and not n.status == plans.ActionStatusEnum.EXECUTABLE and n != exec_plan.init_node]
    real_expl_actions = [n for n in expl_plan.topological_sort() if n.action.name.startswith("_action_")]

    node_mapping = dict(zip(real_expl_actions, real_exec_actions))
    node_mapping.update(zip(real_exec_actions, real_expl_actions))
    committment_dict = {}
    commit_conflicts = set()
    
    for n in exec_plan.nodes_iter():
        if n.is_virtual():
            for eff in n.effects:
                if eff.svar.modality == pddl.mapl.commit:
                    committment_dict[eff.svar.nonmodal()] = (n, eff.svar.modal_args[0])

    for n in expl_plan.nodes_iter():
        if not n.action.name.startswith("_action_"):
            for eff in n.effects:
                if eff.svar.modality == pddl.mapl.commit:
                    if eff.svar.nonmodal() in committment_dict:
                        other_n, other_val = committment_dict[eff.svar.nonmodal()]
                        node_mapping[n] = other_n
                        node_mapping[other_n] = n
                        if other_val != eff.svar.modal_args[0]:
                            commit_conflicts.add(n)

    def changed_effects(n):
        if n not in node_mapping:
            return [], []
        other = node_mapping[n]
        svars = set()
        changed = []
        for f in n.effects:
            svars.add(f.svar)
            if fact_filter(f) and f not in other.effects:
                changed.append(f)
        deleted = []
        for f in other.effects:
            if fact_filter(f) and f.svar not in svars:
                if f.svar.get_type() == pddl.t_boolean:
                    val = pddl.TRUE if f.value == pddl.FALSE else pddl.FALSE
                    deleted.append(pddl.state.Fact(f.svar, val))
                else:
                    deleted.append(pddl.state.Fact(f.svar, pddl.UNKNOWN))
        return changed, deleted

    def get_conflict_link_target(n, eff):
        other = node_mapping[n]
        for succ in exec_plan.successors_iter(other):
            for e in exec_plan[other][succ].itervalues():
                if e.get("svar",None) == eff.svar and e.get("type",None) == "unexpected":
                    if succ in node_mapping:
                        return node_mapping[succ]
                    return expl_plan.goal_node
        return None

    explanations = set()
    written = {}
    commitments = {}
    for n in expl_plan.topological_sort():
        if "new_facts" in n.action.name:
            for eff in n.effects:
                if eff.svar in written and fact_filter(eff):
                    val, pred = written[eff.svar]
                    if val != eff.value and "new_facts" not in pred.action.name:
                        expl_plan.add_edge(pred, n, svar=eff.svar, val=val, type="unexpected")
        changed, deleted = changed_effects(n)
        print n, "changed:", map(str, changed), "deleted:", map(str, deleted)
        for f in chain(changed, deleted):
            succ = get_conflict_link_target(n, f)
            if succ:
                expl_plan.add_edge(n, succ, svar=f.svar, val=f.value, type="repaired")
            for cconds, ceffs in get_causal_relations(n):
                eff_val = None
                for eff in ceffs:
                    if eff.svar == f.svar:
                        eff_val = eff.value
                        break
                if eff_val is not None:
                    negated = (eff_val != f.value)
                    print "changed effect:", eff, negated
                    for c in cconds:
                        print c
                        if c.svar.modality == pddl.mapl.commit:
                            cvar = c.svar.nonmodal()
                            cval = c.svar.modal_args[0]
                            val, pred = commitments.get(cvar, (None,None))
                            if c.value == pddl.FALSE:
                                negated = not negated
                            print "commit:", cvar, cval, val
                        else:
                            cvar, cval  = c
                            val, pred = written.get(cvar, (None,None))
                            print "written:", svar, cval, val
                        if val is not None:
                            if (val == cval and not negated) or (val != cval and negated):
                                expl_plan.add_edge(pred, n, svar=cvar, val=val, type="repaired")
                                explanations.add(pred)

            
        for eff in n.effects:
            written[eff.svar] = (eff.value, n)
            if eff.svar.modality == pddl.mapl.commit:
                commitments[eff.svar.nonmodal()] = (eff.svar.modal_args[0], n)
                print "commitment:", eff

    old_assumptions = set()
    for n in exec_plan.nodes_iter():
        if n != exec_plan.init_node and n.is_virtual():
            old_assumptions |= n.effects

    relevant_expl = set(explanations)
    open_expl = set(explanations)
    while open_expl:
        n = open_expl.pop()
        for pred in expl_plan.predecessors_iter(n):
            if pred != expl_plan.init_node and pred.effects - old_assumptions:
                relevant_expl.add(pred)
                open_expl.add(pred)

    for n in relevant_expl:
        print "relevant:", n

    # requiring_expl = set([exec_plan.goal_node])
    # while requiring_expl:
    #     n = requiring_expl.pop()
    #     for pred in exec_plan.predecessors_iter(link_type='unexpected'):
            

    def node_decorator(node):
        if node.action.name in ("_switch_phase_simulate_execution", "_switch_phase_apply_rules"):
            return {"ignore" : True}
        # if node.status == plans.ActionStatusEnum.EXECUTABLE and node.action.name != 'goal' and node not in final_plan_actions:
        #     return {"ignore" : True}
        if "new_facts" in node.action.name:
            return {"label" : "Observations"}
        # if node in commit_conflicts or node in explanations:
        if node in explanations:
            return {"fillcolor" : "green"}
        if node.is_virtual():
            return {"fillcolor" : "darkslategray2"}
        if "explained" in node.action.name:
            return {'ignore' : True}

    def edge_decorator(n1, n2, data):
        if "new_facts" in n1.action.name and n2 == expl_plan.goal_node:
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].modality == pddl.mapl.commit and n2 == expl_plan.goal_node:
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].function.name in ("phase", "enabled", "started"):
            return {'style' : 'invis', 'label' : ''}
        if data['type'] == 'prevent_threat':
            return {'style' : 'invis', 'label' : ''}
        if data['type'] == 'repaired':
            return {'color' : 'green'}
        
    G = expl_plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.write("plan-explained.dot")
    G = expl_plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.layout(prog='dot')
    G.draw("plan-explained.pdf")
    return relevant_expl
