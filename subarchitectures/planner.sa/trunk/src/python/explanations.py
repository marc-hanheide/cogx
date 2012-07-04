from collections import defaultdict
from itertools import chain
from standalone import pddl, plans, config
from standalone.pddl import types
from standalone.pddl.builtin import t_number 

# some globals that will be used all over the module once set
expl_domain = None
se_condition = None
w = None

ignored_functions = set()

svars_to_check = set()
known_svars = set()

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
                    add_consistence_checks(a)
                    # a.extend_precondition(str2cond("(= (phase) apply_rules)", expl_domain))

def add_consistence_checks(action):
    for eff in action.effect.visit(pddl.visitors.collect_effects):
        if eff.predicate == pddl.mapl.commit:
            term = eff.args[0]
            cond = pddl.LiteralCondition(pddl.mapl.committed, [term], negated=True)
            # cond = pddl.LiteralCondition(compatible, eff.args)
            action.extend_precondition(cond)
                    
def build_operator_for_ground_action(i, action, args):
    action_t = expl_domain.types["action"]
    action_id = "action_" + str(i).zfill(3)
    expl_domain.add_constant(types.TypedObject(action_id, action_t))
    name = "_%s_%s" % (action_id, action.name)
    new_op = pddl.Action(name, [], None, None, expl_domain)
    with action.instantiate(args, expl_domain):
        if action.precondition:
            new_op.precondition = action.precondition.copy(copy_instance=True)
        #new_op.precondition = pddl.Conjunction([]) # TEST
        if action.effect:
            new_op.effect = action.effect.copy(copy_instance=True)

    @pddl.visitors.collect
    def get_cconds(elem, results):
        if isinstance(elem, pddl.effects.ConditionalEffect):
            return elem.condition.visit(pddl.visitors.collect_conditions)
            
    #Commit preconditions: if there is a conditional effect, make sure
    #that we commit to something to prevent "explanation by inaction"
    for eff in pddl.visitors.visit(new_op.effect, get_cconds, []):
        if eff.predicate in (pddl.mapl.commit, pddl.builtin.equals) and not eff.negated:
            term = eff.args[0]
            cond = pddl.LiteralCondition(pddl.mapl.committed, [term], scope=new_op)
            new_op.extend_precondition(cond)
            svars_to_check.add(pddl.state.StateVariable.from_term(term))
            
    enabled_cond = str2cond("(= (enabled) %s)" % action_id, expl_domain)
    # for cond in [se_condition, enabled_cond]:
    for cond in [enabled_cond]:
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
            with r.instantiate(mapping, expl_domain):
                pre = r.precondition.copy(copy_instance=True)
            break
    return pddl.visitors.visit(pre, pddl.visitors.collect_conditions, [])

def possible_compatibility_effect(fact):
    if fact.svar.modality:
        return False
    if isinstance(fact.svar.function, pddl.Predicate):
        return False
    if fact.svar in known_svars:
        return False
    return fact_filter(fact)

def build_operator_for_new_facts(i, node):
    compatibility_conds = set(pddl.state.Fact(f.svar.as_modality(compatible, [f.value]), pddl.TRUE) for f in node.preconds if possible_commit_effect(f))

    compatibility_conds |= set(pddl.state.Fact(f.svar.as_modality(compatible, [f.value]), pddl.TRUE) for f in node.effects if possible_compatibility_effect(f))
    
    node.preconds = set(f for f in node.preconds if possible_cond_effect(f))
    known_svars.update(f.svar for f in node.effects if f.svar.modality is None)

    rule_conditions = sum((state_rule_conditions(f) for f in node.effects if fact_filter(f)), [])
    
    action_t = expl_domain.types["action"]
    action_id = "action_" + str(i).zfill(3)
    expl_domain.add_constant(types.TypedObject(action_id, action_t))
    name = "_%s_%s" % (action_id, node.action.name)
    new_op = pddl.Action(name, [], None, None, expl_domain, None)
    new_op.precondition = pddl.Conjunction(rule_conditions + [f.to_condition() for f in chain(node.preconds, compatibility_conds) if fact_filter(f)], new_op)
    new_op.effect = pddl.ConjunctiveEffect([f.to_effect() for f in node.effects if f not in node.preconds and fact_filter(f)], new_op)
    add_ops = []
    # for i, pre in enumerate(node.preconds):
    #     name = "forfeit_%s_%s-%d" % (action_id, node.action.name, i)
    #     new_op2 = pddl.Action(name, [], None, None, expl_domain, None)
    #     new_op2.effect = pre.to_effect()
    #     # new_op2.precondition = pddl.Conjunction([f.to_condition() for f in node.preconds if fact_filter(f)], new_op2)
    #     new_op2.set_total_cost(pddl.Term(500))
    #     add_ops.append(new_op2)
    
    enabled_cond = str2cond("(= (enabled) %s)" % action_id, expl_domain)
    # for cond in [se_condition, enabled_cond]:
    for cond in [enabled_cond]:
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
    # for phase in ["make_alternative_assumptions", "apply_rules", "simulate_execution", "achieve_goal"]:
    #     expl_domain.add_constant(types.TypedObject(phase, phase_t))
    # aa_condition = str2cond("(= (phase) make_alternative_assumptions)", expl_domain)
    # ar_condition = str2cond("(= (phase) apply_rules)", expl_domain)
    # se_condition = str2cond("(= (phase) simulate_execution)", expl_domain)

    expl_domain.add_constant(types.TypedObject("achieve_goal", action_t))
        
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
            # a.extend_precondition(ar_condition)
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
        op.extend_effect(str2eff("(assign (enabled) achieve_goal)", expl_domain))
    # add action to switch phases
    # for swop in [SWITCH_OP1, SWITCH_OP2]:
    #     switch_action = pddl.parser.Parser.parse_as(swop.splitlines(), pddl.Action, expl_domain)
    #     expl_domain.add_action(switch_action)

def apply_high_p_assumptions(init_state, domain):
    problem = init_state.problem
    def get_p(effects):
        for svar, val in effects.iteritems():
            if svar.function.name == "probability":
                return val.value
        return 1.0

    probs = defaultdict(lambda: 0.0)
    svar_facts = defaultdict(set)

    def ignore_phase_check_fn(fact):
        if fact.svar.function.name == "enabled":
            return True
    
    for action in domain.actions:
        if not action.args:
            continue
        print "----",action.name
        inst_func = action.get_inst_func(init_state, fact_check_fn=ignore_phase_check_fn)
        print map(str, [map(str, list(problem.get_all_objects(a.type))) for a in action.args])
        for mapping in action.smart_instantiate(inst_func, action.args, [list(problem.get_all_objects(a.type)) for a in action.args], problem):
            effects = init_state.get_effect_facts(action.effect)
            p = get_p(effects)
            for svar, val in effects.iteritems():
                if svar.modality == pddl.mapl.commit:
                    fact = pddl.state.Fact(svar.nonmodal(), svar.modal_args[0])
                    probs[fact] = max(probs[fact], p)
                    svar_facts[fact.svar].add(fact)

    new_state = init_state.copy()
    for svar, facts in svar_facts.iteritems():
        if sum(probs[f] for f in facts) > 1.0:
            continue
        for f in facts:
            if probs[f] > 0.91 and probs[f] < 1.0:
                print "set:", f, probs[f]
                new_state.set(f)

    return new_state

def stratify_assumptions(domain):
    all_functions = set()
    all_actions = set()
    conditions_to_actions = defaultdict(set)
    effects_to_actions = defaultdict(set)
    R=defaultdict(lambda: 0)
    @pddl.visitors.collect
    def get_assumptions(elem, results):
        if isinstance(elem, pddl.Literal) and elem.predicate == pddl.mapl.commit:
            return elem.args[0].function
        
    for a in domain.actions:
        if a.name.startswith("_action_") or a.name.startswith("forfeit_action_"):
            continue
        all_actions.add(a)
        for f2 in a.effect.visit(get_assumptions):
            effects_to_actions[f2].add(a)
            all_functions.add(f2)
            for f in a.precondition.visit(get_assumptions):
                conditions_to_actions[f].add(a)
                all_functions.add(f)
                R[f, f2] = 1

    # for j in all_functions:
    #     for i in all_functions:
    #         for k in all_functions:
    #             if min(R[i,j], R[j,k]) > 0:
    #                 R[i,k] = max(R[i,j], R[j,k], R[i,k])

    def succ(f):
        return (x for x in all_functions if (f,x) in R)

    #find strongly connected components
    index = [0]
    indexes = {}
    lowlink = {}
    S = []
    scc = []

    def tarjan_scc(f):
        indexes[f] = index[0]
        lowlink[f] = index[0]
        index[0] += 1
        S.append(f)

        for f2 in succ(f):
            if f2 not in indexes:
                tarjan_scc(f2)
                lowlink[f] = min(lowlink[f], lowlink[f2])
            elif f2 in S:
                lowlink[f] = min(lowlink[f], indexes[f2])

        if lowlink[f] == indexes[f]:
            comp = set([f])
            f2 = S.pop(-1)
            while f != f2:
                comp.add(f2)
                f2 = S.pop(-1)
            # print map(str, comp)
            scc.append(comp)
        
    for f in all_functions:
        if f not in indexes:
            tarjan_scc(f)
                    

    #extract strata
    remaining = set(all_functions)
    layers = []
    
    while remaining:
        stratum = set()
        for j in remaining:
            # print j, [R[i,j] for i in remaining]
            if all(R[i,j] != 1 for i in remaining):
                layers.append([j])
                stratum.add(j)
                # print "layer:", j
                
        if not stratum:
            for comp in scc:
                if comp & remaining:
                    other_remaining = remaining - comp
                    if all(R[i,j] != 1 for i in other_remaining):
                        layers.append(comp)
                        stratum |= comp
                        # print "layer:", map(str, comp)

        remaining -= stratum

    layers_for_actions = [list() for x in layers]
    for i, l in reversed(list(enumerate(layers))):
        for f in l:
            for a in conditions_to_actions[f]:
                if a in all_actions:
                    # print "add:", a.name, "due to", f.name
                    layers_for_actions[i].append(a)
                    all_actions.remove(a)
    for a in all_actions:
        layers_for_actions.insert(0, [a])

    phase_t = expl_domain.types["phase"]
    action_t = expl_domain.types["action"]
    phase = 0
    phase_obj = types.TypedObject("apply_rules_" + str(phase).zfill(3), action_t)
    expl_domain.add_constant(phase_obj)
    for l in layers_for_actions:
        if not l:
            continue
        
        if phase != 0:
            num = str(phase).zfill(3)
            new_phase_obj = types.TypedObject("apply_rules_%s" % num , action_t)
            expl_domain.add_constant(new_phase_obj)
            switch = pddl.Action("_switch_rules_%s" % num, [], None, None, expl_domain)
            switch.precondition = pddl.Builder(switch).cond("=", ("enabled",), phase_obj)
            switch.effect = pddl.Builder(switch).effect("assign", ("enabled",), new_phase_obj)
            expl_domain.add_action(switch)
            phase_obj = new_phase_obj
        
        for a in l:
            a.set_parent(expl_domain)
            a.extend_precondition(pddl.Builder(a).cond("=", ("enabled",), phase_obj))

        phase += 1
        
    switch = pddl.Action("_switch_phase_simulate_execution", [], None, None, expl_domain)
    switch.precondition = pddl.Builder(switch).cond("=", ("enabled",), phase_obj)
    switch.effect = pddl.Builder(switch).effect("assign", ("enabled",), "action_000")
    expl_domain.add_action(switch)
        
        


def build_explanation_problem(problem, last_plan, init_state, observed_state):
    assert expl_domain is not None
    p = problem.copy(newdomain=expl_domain)
    p.objects = set()
    
    stratify_assumptions(expl_domain)
    high_p_init_state = apply_high_p_assumptions(init_state, expl_domain)

    p.init = [f.to_init() for f in high_p_init_state.iterfacts()]
    p.init.append(pddl.Builder(p).init("=", ("enabled",), "apply_rules_000"))
    # p.init.append(pddl.Builder(p).init("=", ("enabled",), "action_000"))

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
    if not isinstance(goal, pddl.Conjunction):
        goal = pddl.Conjunction([goal])
    # goal.parts.append(str2cond("(= (phase) achieve_goal)", expl_domain))
    goal.parts.append(str2cond("(= (enabled) achieve_goal)", expl_domain))
    p.goal = goal
    # p.goal = pddl.Conjunction([str2cond("(= (phase) achieve_goal)", expl_domain)])
    return p
    

def handle_failure(last_plan, problem, init_state, observed_state, expl_rules_fn, cp_task, component):
    global w
    w = pddl.mapl.MAPLWriter()

    if "started" in problem.domain.predicates:
        ignored_functions.add(problem.domain.predicates["started"][0])

    sorted_plan = last_plan.topological_sort()

    known_svars.update(f.svar for f in init_state.iterfacts() if f.svar.modality is None)
    
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
        result, expl_plan  = postprocess_explanations(plan, last_plan)
        component.verbalise("I have now found a possible explanation for the failure.")
        return result, expl_plan
    else:
        component.verbalise("I'm sorry. I can't explain what went wrong.")
        print "No explanations found"
        return None, None


def get_causal_relations(node):
    @pddl.visitors.collect
    def get_ceffs(elem, results):
        if isinstance(elem, pddl.effects.ConditionalEffect):
            return elem

    if isinstance(node, plans.DummyNode):
        return

    with node.action.instantiate(node.full_args, parent=expl_domain):
        for ceff in pddl.visitors.visit(node.action.effect, get_ceffs, []):
            conds = ceff.condition.visit(pddl.visitors.collect_literals)
            effs = ceff.effect.visit(pddl.visitors.collect_literals)
            c_facts = [pddl.state.Fact.from_literal(l) for l in conds]
            e_facts = [pddl.state.Fact.from_literal(l) for l in effs]
            yield c_facts, e_facts
    
      
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
                        # print "unexpected: %s = %s instead of %s" % (str(eff.svar), str(eff.value), str(val))
                        expl_plan.replace_link(pred, n, eff.svar, eff.value, "unexpected")
        changed, deleted = changed_effects(n)
        print n, "changed:", map(str, changed), "deleted:", map(str, deleted)
        for f in chain(changed, deleted):
            succ = get_conflict_link_target(n, f)
            if succ:
                expl_plan.replace_link(n, succ, f.svar, f.value, "repaired")
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
                                expl_plan.replace_link(pred, n, cvar, val, "explanation")
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
            if pred.action.name.startswith("_switch"):
                continue
            if pred != expl_plan.init_node and pred.effects - old_assumptions:
                relevant_expl.add(pred)
                open_expl.add(pred)

    for n in relevant_expl:
        def fact_transform(f):
            if f.value.is_instance_of(pddl.t_object) and f.value.name.startswith("unknown-"):
                return pddl.state.Fact(f.svar, pddl.UNKNOWN)
            return f
        n.effects = set(fact_transform(f) for f in n.effects)
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
        if "new_facts" in n2.action.name and n1 == expl_plan.init_node:
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].modality == pddl.mapl.commit and n2 == expl_plan.goal_node:
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].modality == pddl.mapl.committed and data['val'] == pddl.FALSE:
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].function.name in ("phase", "enabled", "started", "done"):
            return {'style' : 'invis', 'label' : ''}
        if data['type'] == 'prevent_threat':
            return {'style' : 'invis', 'label' : ''}
        if data['type'] == 'repaired':
            return {'color' : 'green'}
        if data['type'] == 'explanation':
            return {'color' : 'green', 'style' : 'dashed'}
        
    G = expl_plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.write("plan-explained.dot")
    G = expl_plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.layout(prog='dot')
    G.draw("plan-explained.pdf")
    return relevant_expl, expl_plan
