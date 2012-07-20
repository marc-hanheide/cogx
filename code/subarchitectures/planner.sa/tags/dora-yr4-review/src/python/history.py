import collections, re, time
import fake_cast_state
from standalone import task, pddl, plans, plan_postprocess
from autogen import Planner

def first(fn, seq):
    for x in seq:
        if fn(x):
            return x
    return None

def slice_goals_from_problem(problem):
    softgoals = []

    @pddl.visitors.copy
    def goal_visitor(elem, results):
        if isinstance(elem, pddl.conditions.PreferenceCondition):
            softgoals.append((elem.penalty, elem.cond))
            return False

    hard_goal = problem.goal.visit(goal_visitor)
    slice_goals = [Planner.Goal(importance=-1.0, deadline=-1, goalString=hard_goal.pddl_str(), isInPlan=False)]
    for pri, goal in softgoals:
        slice_goals.append(Planner.Goal(importance=pri, deadline=-1, goalString=goal.pddl_str(), isInPlan=False))

    return slice_goals
    

def parse_history_new(history_fn, domain):
    p = pddl.parser.Parser.parse_file(history_fn)
    it = iter(p.root)
    it.get(":log")

    def history_iter():
        action_states = {}

        def is_auto_exec(a):
            if a == "init":
                return True
            if a.startswith("__"):
                return True
        
        def get_action_status(a):
            #get actions by prefix, as the :action events only specify partial parameter lists
            #can break, but better than nothing.
            keys = []
            for k in a.split():
                keys.append(k)
                k_new = " ".join(keys)
                if k_new in action_states:
                    return action_states[k_new]
            return plans.ActionStatusEnum.EXECUTABLE
        
        last_state = None
        last_plan_state = None
        last_plan = None
        dt = False
        dt_action = None
        for elem in it:
            jt = iter(elem)
            tag = jt.get('terminal').token.string
            ts = jt.get()
            if tag == ":state":
                last_state = jt.get()
            elif tag == ":plan":
                if last_plan is not None:
                    plan = [(a, get_action_status(a)) for a in last_plan]
                    problem = pddl.problem.Problem.parse(last_plan_state, domain)
                    yield problem, plan

                action_states.clear()
                last_plan = []
                for elems in jt:
                    if elems.is_terminal() and elems.token.string == ":notfound":
                        last_plan = []
                        break
                    action = " ".join(e.token.string for e in elems)
                    last_plan.append(action)
                    if is_auto_exec(action):
                        action_states[action] = plans.ActionStatusEnum.EXECUTED
                last_plan_state = last_state
            elif tag == ":action" and not dt:
                #reconstruct action status at the end of plan execution
                action, status = tuple(jt.get(list))
                action = " ".join(e.token.string for e in action)
                status = plans.ActionStatusEnum.__dict__[status.token.string.upper()]
                action_states[action] = status
            elif tag == ":dt":
                dtstatus = jt.get('terminal').token.string
                if dtstatus == ":started":
                    #get first nonexecuted action that (probably) triggered DT
                    dt_action = first(lambda x: get_action_status(x) == plans.ActionStatusEnum.EXECUTABLE, last_plan)
                    action_states[dt_action] = plans.ActionStatusEnum.IN_PROGRESS
                    # print "dt action is:", dt_action
                    dt = True
                elif dtstatus == ":cancelled":
                    action_states[dt_action] = plans.ActionStatusEnum.FAILED
                    dt = False
                else:
                    action_states[dt_action] = plans.ActionStatusEnum.EXECUTED
                    dt = False
                # print dt_action, action_states[dt_action]

                    
        if last_plan is not None:
            plan = [(a, get_action_status(a)) for a in last_plan]
            problem = pddl.problem.Problem.parse(last_plan_state, domain)
            yield problem, plan
        elif last_state is not None:
            problem = pddl.problem.Problem.parse(last_state, domain)
            yield problem, []
            
    return history_iter()

    
def parse_history_old(history_fn, domain):

    def read_file(fn):
        read_problem=True
        problem=[]
        plan=[]
        def get_section(lines, endstr):
            for l in lines:
                if l == endstr:
                    return
                yield l
                
        lines = (l.strip() for l in open(fn))
        while True:
            problem = list(get_section(lines, "END_PROBLEM"))
            if not problem:
                return
            plan = list(get_section(lines, "END_PLAN"))
            poplan = list(get_section(lines, "END_POPLAN"))
            yield problem, plan
        

    PDDL_REXP = re.compile("\((.*)\)")
    def extract_action(line):
        action, status = line.split(",")
        status = plans.ActionStatusEnum.__dict__[status.strip()]
        return PDDL_REXP.search(action).group(1).lower().strip(), status

    def history_iter():
        for prob_str, plan_str in read_file(history_fn):
            t0 = time.time()
            problem = pddl.parser.Parser.parse_as(prob_str, pddl.Problem, domain)
            print "parsing problem: %.2f" % (time.time() - t0)

            plan = [extract_action(l) for l in plan_str]
            yield problem, plan
    return history_iter()
        
def load_history(history_fn, domain, component=None, consistency_cond = None):
    
    # log.info("Loading history: %s.", history_fn)
    _task = None
    init_state = None

    try:
        history = parse_history_new(history_fn, domain)
    except pddl.parser.ParseError as e:
        try:
            history = parse_history_old(history_fn, domain)
        except:
            raise e
        
    
    for problem, plan in history:
        t0 = time.time()
        state = fake_cast_state.FakeCASTState(problem, domain, component=component, consistency_cond=consistency_cond)
        print "state generation: %.2f" % (time.time() - t0)
        if init_state is None:
            init_state = state

        slice_goals = slice_goals_from_problem(problem)
        cp_problem, cp_domain, goaldict = state.to_problem(slice_goals, deterministic=True)
        print "problem re-generation: %.2f" % (time.time() - t0)
        state.goals = slice_goals

        _task = task.Task(0, cp_problem)
        _task.set_state(state.state)
        actions = []
        actions_status = {}
        for action, status in plan:
            actions_status[action] = status
            if action in ("init", "goal"):
                continue
            actions.append(action)

        if actions:
            plan = plan_postprocess.make_po_plan(list(enumerate(actions)), _task)
            for n in plan.nodes_iter():
                key = "%s %s" % (n.action.name, " ".join(a.name for a in n.full_args))
                n.status = actions_status[key.strip()]
                if n.is_virtual() or n == plan.init_node:
                    n.status = plans.ActionStatusEnum.EXECUTED
        else:
            plan = None
        print "plan post-processing: %.2f" % (time.time() - t0)
        yield plan, state


