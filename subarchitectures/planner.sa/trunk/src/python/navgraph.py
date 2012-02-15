#!/usr/bin/env python
import re, sys, time
from pygraphviz import AGraph

import standalone.task, standalone.planner
from standalone import plans, pddl
import standalone.globals as global_vars
import autogen
import history
import networkx

global_vars.config.__dict__['enable_switching_planner'] = True

room_colors = ["red", "green", "blue", "yellow", "orange", "cyan", "black"]

assert len(sys.argv) == 3, """Call 'navgraph.py domain.pddl problem.pddl' for a single planner call"""
domain_fn, problem_fn = sys.argv[1:]

print "Loading domain..."
dom = pddl.load_domain(domain_fn)

t_room = dom.types["room"]
t_place = dom.types["place"]
t_robot = dom.types["robot"]

f_connected = dom.predicates['connected'][0]
f_room = dom.functions['in-room'][0]
f_cat = dom.functions['category'][0]
f_type = dom.functions['placestatus'][0]
f_attached = dom.functions['leads_to_room'][0]
f_pos = dom.functions.get('is-in', [t_robot])

c_placeholder = dom["placeholder"]
c_place = dom["trueplace"]

target_category = "kitchen"

colordict = {}
planner = None

def get_room_color(room):
    if room is None:
        return 'white'
    if room not in colordict:
        col = room_colors.pop(0)
        colordict[room] = col

    return colordict[room]

def show_step(state, plan, domain=None):
    if domain is None:
        domain = dom

    robot = None
    robot_pos = None

    rooms = [o for o in state.problem.objects if o.is_instance_of(t_room)]
    nodes = [o for o in state.problem.objects if o.is_instance_of(t_place)]
    cat = state.problem[target_category]

    filled_placeholders = set()
    cat_probs = {}

    for svar, val in state.iteritems():
        if svar.function == f_attached:
            n = svar.args[0]
            filled_placeholders.add(n)
            if svar.args[1] == cat:
                p = val[pddl.TRUE]
                cat_probs[n] = p
        if svar.function == f_cat and val.value != pddl.UNKNOWN:
            print svar, val
            r = svar.args[0]
            p = val[cat]
            cat_probs[r] = p
            
        if svar.function == f_pos:
            robot = svar.args[0]
            robot_pos = val.value

    CAST_RE = re.compile("[^_]*_(_*[^_]+)_(_*[^_]+)")
    def get_name(n):
        match = CAST_RE.search(n.name)
        if not match:
            return n.name
        s1 = match.group(1)
        s2 = match.group(2)
        if s1[0] == '_':
            s1 = s1[1:].upper()
        if s2[0] == '_':
            s2 = s2[1:].upper()
        return "%s:%s" % (s1, s2)

    def get_val(func, obj):
        svar = pddl.state.StateVariable(func, [obj])
        if svar not in state or state[svar] == pddl.UNKNOWN:
            return None
        if isinstance(state[svar], pddl.TypedObject):
            return state[svar]
        return state[svar].value

    G = AGraph(directed=True, strict=False, overlap='scale', sep=.1)

    for n in nodes:
        attrs = {}
        room = get_val(f_room, n)
        typ = get_val(f_type, n)
        attrs['label'] = get_name(n)
        attrs['style'] = 'filled'
        if typ == c_place:
            attrs['fillcolor'] = get_room_color(room)

            attrs['shape'] = 'doublecircle'
        elif typ == c_placeholder:
            if n in filled_placeholders:
                attrs['fillcolor'] = 'grey'
            attrs['shape'] = 'circle'
        else:
            assert False
        if room is not None and room in cat_probs:
            attrs['label'] = "%s\\nr:%.2f" % (get_name(n), cat_probs[room])
        elif n in cat_probs and typ == c_placeholder:
            attrs['label'] = "%s\\n%.2f" % (get_name(n), cat_probs[n])

        if n == robot_pos:
            attrs['penwidth'] = '3'
            # attrs['color'] = 'green'

        G.add_node(n, **attrs)

    for svar, val in state.iteritems():
        if svar.function != f_connected or val.value != pddl.TRUE:
            continue
        n1, n2 = svar.args
        attrs = {'dir' : 'none'}
        G.add_edge(n1, n2, **attrs)

    if plan:
        for pnode in plan:
            if pnode.action.name not in ("move", "move_direct"):
                continue
            to_node = pnode.full_args[1]
            from_node = pnode.full_args[2]
            attrs = {'dir' : 'forward'}
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                attrs['color'] = 'green'
            elif pnode.status == plans.ActionStatusEnum.FAILED:
                attrs['color'] = 'red'
            else:
                attrs['color'] = 'blue'

            G.add_edge(from_node, to_node, **attrs)

    G.layout(prog='neato')
    G.draw("nodes.pdf")

def replan(cast_state):
    global planner
    if planner is None:
        planner = standalone.planner.Planner()

    t0 = time.time()
    cp_problem, cp_domain, _ = state.to_problem(state.goals, deterministic=True)
    print "problem creation took: %.2f sec" % (time.time() - t0)

    task = standalone.task.Task(0, cp_problem)
    task.deadline = 211
    task.set_state(state.state)
    planner.register_task(task)
    task.replan()
    print "planning took: %.2f sec" % (time.time() - t0)

    return task.get_plan()

def print_plan(plan):
    if plan is None:
        print "No plan found"
        return
    
    for pnode in plan.topological_sort():
        if pnode in (plan.goal_node, plan.init_node):
            continue
        if pnode.status == plans.ActionStatusEnum.EXECUTED:
            print "  |",
        elif pnode.status == plans.ActionStatusEnum.FAILED:
            print "  X",
        else:
            print "   ",
        print pnode

print "Loading problem..."
try:
    prob = pddl.load_problem(problem_fn, dom)
    state = pddl.prob_state.ProbabilisticState.from_problem(prob)
    show_step(staet, None)
except:
    history_iter = history.load_history(problem_fn, dom)
    h_list = []
    t = 0
    max_t = None
    stop = False
    while not stop:
        while t >= len(h_list):
            try:
                h_list.append(history_iter.next())
            except StopIteration, e:
                max_t = len(h_list)-1
                t = max_t
        plan, state = h_list[t]
        show_step(state.prob_state, plan)
        print
        print "Timestep: %d" % t
        print
        print_plan(plan)
        print
            
        handled = False
        while not handled:
            print "(p)rev, (N)ext, (r)eplan, (q)uit: ",
            c = raw_input()
            if c == 'p' and t > 0:
                t -= 1
                handled = True
            elif c in ('n', '') and (max_t is None or t < max_t):
                t += 1
                handled = True
            elif c == 'r':
                print "Replanning..."
                plan = replan(state)
                show_step(state.prob_state, plan)
                print
                print "Timestep: %d" % t
                print
                print_plan(plan)
                print

            elif c == 'q':
                stop = True
                handled = True
            else:
                try:
                    new_t = int(c)
                    if max_t is None or new_t <= max_t:
                        t = new_t
                        handled = True
                    
                except ValueError:
                    pass
                
    
