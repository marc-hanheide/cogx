from collections import defaultdict
import itertools
import pddl
from pddl import state
import plans

def merge_plans(_plans, init_state, final_state):
    domain = init_state.problem.domain
    
    nodedict = {}
    latest_node = {}
    all_nodes = defaultdict(set)
    virtual_mappings = {}

    def add_node(n):
        key = (n.action.name,)+tuple(virtual_mappings.get(a,a) for a in  n.full_args)
        nodedict[key] = n
        all_nodes[key].add(n)

    def get_node(n):
        key = (n.action.name,)+tuple(virtual_mappings.get(a,a) for a in  n.full_args)
        if key not in nodedict:
            nodedict[key] = n
        # latest_node[key] = n
        all_nodes[key].add(n)
        return nodedict[key]

    # def latest(n):
    #     key = (n.action.name,)+tuple(n.full_args)
    #     return latest[key]

    plan_dict = {}
    written_dict = {}
    def get_incoming_links(n):
        if n not in plan_dict:
            return
        plan = plan_dict[n]
        for pred in plan.predecessors_iter(n):
            for e in plan[pred][n].itervalues():
                yield pred, e["svar"], e["val"], e["type"]

    def has_link(plan, n1, n2, svar, val, type):
        if n2 not in plan[n1]:
            return False
        for e in plan[n1][n2].itervalues():
            if e["svar"] == svar and e["val"] == val and e["type"] == type:
                return True
        return False

    def all_objects(pnode):
        for f in itertools.chain(pnode.preconds, pnode.effects):
            for obj in itertools.chain(f.svar.args, f.svar.modal_args, [f.value]):
                yield obj
        for obj in pnode.full_args:
            yield obj

    def state_diff(old, new):
        facts = []
        for f in new.iterfacts():
            if f not in old:
                facts.append(f)
        for f in old.iterfacts():
            if f.svar not in new and f.svar.function not in (pddl.builtin.total_cost,):
                if f.svar.get_type() == pddl.t_boolean:
                    facts.append(pddl.state.Fact(f.svar, pddl.FALSE))
                # else:
                #     facts.append(pddl.state.Fact(f.svar, pddl.UNKNOWN))
        return facts

    virtual_p = domain.predicates['is-virtual'][0] if 'is-virtual' in domain.predicates else None

    def is_virtual_fact(fact):
        if "virtual" in fact.svar.function.name:
            return True
        return False

    virtual_objects = []
    virtual_facts = defaultdict(set)
    used_virtual_objects = []
    def canonical_vo(o):
        if o in virtual_objects:
            return pddl.TypedObject("virtual-%s" % str(o.type), o.type)
        return o

    def add_virtual_objects(init_node):
        for f in init_node.effects:
            if f.svar.function == virtual_p and f.svar.args[0] not in virtual_objects:
                virtual_objects.append(f.svar.args[0])

    def add_virtual_object_facts(fact):
        if is_virtual_fact(fact):
            return
        for o in fact.svar.args:
            if o in virtual_objects:
                vo_svar = state.StateVariable(fact.svar.function, map(canonical_vo, fact.svar.args), fact.svar.modality, map(canonical_vo, fact.svar.modal_args))
                virtual_facts[o].add(state.Fact(vo_svar, canonical_vo(fact.value)))

    def objects_match(new, old):
        d = dict(virtual_facts[old])
        if new.type != old.type:
            return False
        for f in virtual_facts[new]:
            if d.get(f.svar, f.value) != f.value:
                # print "mismatch:", f, d[f.svar]
                return False
        return True

    def mapped_svar(svar):
        return state.StateVariable(svar.function, [virtual_mappings.get(o,o) for o in svar.args],\
                                       svar.modality, [virtual_mappings.get(o,o) for o in svar.modal_args])

    def mapped_fact(fact):
        return state.Fact(mapped_svar(fact.svar), virtual_mappings.get(fact.value, fact.value))

    def mapped_node(pnode):

        new = plans.PlanNode(pnode.action, [virtual_mappings.get(o,o) for o in pnode.args], pnode.time, pnode.status)

        new.preconds = set(map(mapped_fact, pnode.preconds))
        new.effects = set(map(mapped_fact, pnode.effects))
        new.original_preconds = set(map(mapped_fact, pnode.original_preconds))

        new.full_args = [virtual_mappings.get(o,o) for o in pnode.full_args]
        return new


    plan = plans.MAPLPlan(init_state, init_state.problem.goal)
    init_problem = init_state.problem.copy()
    current_state = init_state.copy()
    get_node(plan.init_node)
    get_node(plan.goal_node)

    written = {}
    observed = {}
    executed = []
    unexecuted = []
    last_plan = None

    for i, p in enumerate(_plans):
        if not isinstance(p, plans.MAPLPlan):
            continue
        G = p.to_dot()
        G.layout(prog='dot')
        G.draw("plan%d.pdf" % i)

        last_plan = p
        for n in p.topological_sort():
            if n.action.name == 'init':
                add_virtual_objects(n)

            if n.action.name == 'init' or n.is_virtual():
                for f in n.effects:
                    add_virtual_object_facts(f)

            for o in all_objects(n):
                if o not in init_problem:
                    init_problem.add_object(o)
            plan_dict[n] = p
            if n.status in (plans.ActionStatusEnum.EXECUTED, plans.ActionStatusEnum.FAILED):
                executed.append(n)
            else:
                unexecuted.append(n)

    current_state_node = plans.DummyNode("init", [], 0, plans.ActionStatusEnum.EXECUTED)
    current_state_node.effects = set(final_state.iterfacts())
    executed.append(current_state_node)

    # print "virtual objects:", map(str, virtual_objects)
    for o in virtual_objects:
        for uo in used_virtual_objects:
            if objects_match(o, uo):
                virtual_mappings[o] = uo
                # print "map", o, "to", uo
                break
        if o not in virtual_mappings:
            used_virtual_objects.append(o)

    # print "used_vos", map(str, used_virtual_objects)

    def add_links(n, new_n):
        print "adding links:", new_n
        for pred, svar, val, type in get_incoming_links(n):
            svar = mapped_svar(svar)
            val = virtual_mappings.get(val, val)
            # if p.action.name == "init":
            #     print p, p in init_dict, id(p)
            if False:#pred in init_dict:
                new_p = init_dict[pred] #What about mapping???
                print "linking %s = %s to %s instead of init" % (str(svar), str(val), str(new_p))
            else:
                new_p = get_node(pred)
            print "    old link:", pred, svar, val, type
            if new_p == plan.init_node and svar in written and not n.is_virtual():
                real_p, expected_val = written[svar]
                print "        ", real_p
                new_p = real_p
                if val == expected_val:
                    type = "depends"
                    # plan.add_edge(real_p, new_n, svar=svar, val=val, type = "depends")
                else:
                    type = "unexpected"
                    # plan.add_edge(real_p, new_n, svar=svar, val=val, type = "unexpected")
            if not has_link(plan, new_p, new_n, svar, val, type):
                print "    new link:", new_p, new_n, " (%s = %s)" % (svar, val)
                plan.add_edge(new_p, new_n, svar=svar, val=val, type=type)

    init_dict = {}
    used_objects = set()
    prev_init_node = None
    conflicting_svars = defaultdict(set)
    prev_was_init = False
    i = 0
    for n in executed:
        n.time = i
        i += 1
        mapped_n = mapped_node(n)
        print "===", n
        if n.is_virtual() or n.action.name == "goal":
            new_n =  get_node(mapped_n)
            prev_was_init = False
        elif n.action.name == "init" and prev_init_node is None:
            new_n =  get_node(mapped_n)
            prev_init_node = new_n
            init_dict[n] = new_n
            print "insert init", id(n)
            prev_was_init = True
        elif n.action.name == "init":
            if prev_was_init:
                continue
            orig_plan = plan_dict.get(n)
            if orig_plan:
                written_dict[orig_plan] = written.copy()
            new_st = pddl.state.State(mapped_n.effects, init_problem)
            # extstate = new_st.get_extended_state()
            # print map(str, 
            diff_facts = state_diff(current_state, new_st)
            new_n = plans.DummyNode("new_facts-%d" % i, [], i, n.status)
            # print [str(f) for f in diff_facts if f.svar.modality is None]
                
            new_n.effects = set(diff_facts)
            new_n.preconds = set()
            add_node(new_n)
            for eff in new_n.effects:
                print "effect:", eff
                if eff.svar in written:
                    if eff.svar in observed:
                        # If we observed an (expected) effect, and subsequently this effect is overwritten by an unexpected event,
                        # link to the last observation, not the original action that caused the effect.
                        # Not doing that would imply that the original action had failed, which we know to be false.
                        real_p, expected_value = observed[eff.svar]
                    else:
                        real_p, expected_value = written[eff.svar]
                        
                    observed_value = new_st.get_extended_state([eff.svar])[eff.svar] #handle axioms
                    print "%s: provided by %s, expected %s, observed %s" % (eff.svar, real_p, expected_value, observed_value)

                    if observed_value == expected_value: 
                        continue
                    assert eff.value != expected_value
                    if not real_p.action.name.startswith("new_facts"):
                        plan.add_edge(real_p, new_n, svar=eff.svar, val=expected_value, type="unexpected")
                        print "unexpected link:", real_p, new_n
                        new_n.preconds.add(eff)
                        conflicting_svars[(real_p, eff.svar)].add(eff.svar)

            for svar, val in mapped_n.effects:
                observed[svar] = new_n, val
                
            plan.add_edge(prev_init_node, new_n, type="order")
            print "init link:", prev_init_node, new_n
            init_dict[n] = new_n
            print "insert init", id(n)
            current_state = new_st
            prev_was_init = True
            prev_init_node = new_n
        else:
            for eff in mapped_n.effects:
                current_state.set(eff)
            add_node(mapped_n)
            new_n = mapped_n
            prev_was_init = False

        if new_n not in plan:
            print "add:", new_n
            plan.add_node(new_n)

        add_links(n, new_n)
        for f in itertools.chain(new_n.preconds, new_n.effects):
            for o in itertools.chain(f.svar.args, f.svar.modal_args, [f.value]):
                used_objects.add(o)

        for svar, val in new_n.effects:
            written[svar] = (new_n, val)
            if svar in observed:
                _, expected = observed[svar]
                if expected != val:
                    print "overwriting observation:", svar
                    del observed[svar]


    written.clear()
    for n in unexecuted:
        orig_plan = plan_dict[n]
        written = written_dict.get(orig_plan, {})
        
        n.time = i
        i += 1
        new_n = get_node(mapped_node(n))
        if new_n in plan:
            print "trying to reuse node:", new_n
            # avoid cycles caused by duplicate actions
            successors = plan.succ_closure(new_n)
            # print "successors:", map(str, successors)
            for p, _, _, _ in get_incoming_links(n):
                new_p = get_node(mapped_node(p))
                # print "checking predecessor:", new_p
                if new_p in successors:
                    # print "cycle!"
                    new_n = mapped_node(n)
                    #reusing node would cause cycle, add new node
                    print "cycle, add unexec:", new_n
                    plan.add_node(new_n)
                    break
        else:
            print "add unexec:", new_n
            plan.add_node(new_n)

        add_links(n, new_n)
        for svar, val in new_n.effects:
            written[svar] = (new_n, val)

    G = plan.to_dot() # a bug in pygraphviz causes write() to delete all node attributes when using subgraphs. So create a new graph.
    G.layout(prog='dot')
    G.draw("plan.pdf")

    redundant = set()
    r_plan = plan.topological_sort()
    r_plan.reverse()
    for n in r_plan:
        if n.status == plans.ActionStatusEnum.EXECUTABLE and n != plan.goal_node:
            # print n
            succ_iter = list(plan.successors_iter(n, link_type = ("depends", "unexpected")))
            # print "    ",map(str, succ_iter)
            if all(succ in redundant or succ.status != plans.ActionStatusEnum.EXECUTABLE for succ in succ_iter):
                # print "redundant", n
                redundant.add(n)

    for n in redundant:
        plan.remove_node(n)

    final_plan_actions = set(get_node(n) for n in last_plan.nodes_iter()) if last_plan else set()

    merged_plan_to_dot(plan, "plan")

    for o in used_objects:
        if o not in init_problem:
            init_problem.add_object(o)

    merged_init_state = pddl.state.State([mapped_fact(f) for f in init_state.iterfacts()], init_problem)
    merged_final_state = pddl.state.State([mapped_fact(f) for f in final_state.iterfacts() if not is_virtual_fact(f)], init_problem)

    print merged_init_state

    return plan, merged_init_state, merged_final_state

def merged_plan_to_dot(plan, fn):
    def node_decorator(node):
        if node.action.name.startswith("new_facts"):
            return {#"label" : "Observations",
                    "fillcolor" : "grey80"}
        # if node.status == plans.ActionStatusEnum.EXECUTABLE and node.action.name != 'goal' and node not in final_plan_actions:
        #     return {"ignore" : True}
        if node.is_virtual():
            return {"fillcolor" : "darkslategray2"}

    def edge_decorator(n1,n2, data):
        if data['type'] == 'order':
            return {'style' : 'invis', 'label' : ''}
        if data['svar'].function.name in ('started', 'done'):
            return {'style' : 'invis', 'label' : ''}
        # cval = conflicting_svars.get((n1, data['svar']), None)
        # if cval is not None and cval != data['val'] :
        #     return {'color' : 'red'}
        if data['type'] == 'unexpected':
            return {'color' : 'red'}

    G = plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.write("%s.dot" % fn)
    G = plan.to_dot(node_deco=node_decorator, edge_deco=edge_decorator) 
    G.layout(prog='dot')
    G.draw("%s.pdf" %fn)
    
