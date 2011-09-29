from standalone import pddl, plans, config
from standalone.pddl import types
from standalone.pddl.builtin import t_number 

# some globals that will be used all over the module once set
expl_domain = None
se_condition = None

def str2cond(fstr, scope):
    return pddl.parser.Parser.parse_as([fstr], pddl.conditions.Condition, scope)

def str2eff(fstr, scope=None):
    return pddl.parser.Parser.parse_as([fstr], pddl.effects.Effect, scope)

def add_explanation_rules(expl_rules_fn):
    global expl_domain
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
            expl_domain.actions.append(pddl.Action.parse(j.reset(), expl_domain))
        elif type == ":init-rule":
            expl_domain.init_rules.append(pddl.InitRule.parse(j.reset(), expl_domain))
        else:
            raise ParseError(type, "Unknown section identifier: '%s'." % type.string)

def build_operator_for_ground_action(i, action, args):
    action_t = expl_domain.types["action"]
    action_id = "action%d" % i
    expl_domain.add_constant(types.TypedObject(action_id, action_t))
    name = "_"+action_id
    new_op = pddl.Action(name, [], None, None, None, None)
    action.instantiate(args, expl_domain)
    if action.precondition:
        new_op.precondition = action.precondition.copy(copy_instance=True)
    if action.effect:
        new_op.effect = action.effect.copy(copy_instance=True)
    enabled_cond = str2cond("(= (enabled) %s)" % action_id, expl_domain)
    for cond in [se_condition, enabled_cond]:
        new_op.extend_precondition(cond)
    enabled_eff = str2eff("(assign (enabled) %s)" % action_id, expl_domain)
    return new_op, enabled_eff

def build_explanation_domain(last_plan, problem, expl_rules_fn):
    global se_condition, expl_domain
    domain_orig = problem.domain
    expl_domain = domain_orig.copy_skeleton()
    expl_domain.name += "-explanations"
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
    # add objects from problem to domain as constants
    for obj in problem.objects:
        expl_domain.add_constant(obj)
    for phase in ["apply_rules", "simulate_execution"]:
        expl_domain.add_constant(types.TypedObject(phase, phase_t))
    ar_condition = str2cond("(= (phase) apply_rules)", expl_domain)
    se_condition = str2cond("(= (phase) simulate_execution)", expl_domain)
   # extract actions from old plan
    last_action = None
    i = 0
    # iterate through plan, create action constants to enfore simulated re-execution during monitoring
    for n in last_plan:
        a = n.action
        if isinstance(a, plans.DummyAction):
            continue
        i += 1
        a2, enabled_last = build_operator_for_ground_action(i, a, n.full_args)
        if last_action:
            last_action.extend_effect(enabled_last)
        last_action = a2
        expl_domain.add_action(a2)
    add_explanation_rules(expl_rules_fn)
    # enforce phases in precondition
    # add action to switch phases
    
def build_explanation_problem(problem, init_state, observed_state):
    facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in observed_state.iterfacts() if not f.value.is_instance_of(t_number)]
    goal = pddl.Conjunction(facts)
    assert expl_domain is not None
    p = problem.copy(newdomain=expl_domain)
    p.init = init_state
    p.goal = goal
    return p
    

def handle_failure(last_plan, problem, init_state, observed_state, expl_rules_fn, cp_task):
    w = pddl.mapl.MAPLWriter()

    build_explanation_domain(last_plan, problem, expl_rules_fn)
    expl_problem = build_explanation_problem(problem, init_state, observed_state)

    # print "\n".join(w.write_domain(expl_domain))
    #print "\n".join(w.write_problem(expl_problem))
    
    cp_task.mapltask = expl_problem
    cp_task.set_state(init_state.state)
    cp_task.set_plan(None)
    cp_task.mark_changed()
    cp_task.replan()
    plan = cp_task.get_plan()
    print plan
