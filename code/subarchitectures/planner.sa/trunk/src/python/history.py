import re, time
import fake_cast_state
from standalone import task, pddl, plans, plan_postprocess
from autogen import Planner

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
    

def load_history(history_fn, domain, component=None, consistency_cond = None):

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
        
        # for l in lines:
        #     if l == 
        #         read_problem = False
        #     elif l == "END_PLAN":
        #         yield problem, plan
        #         problem = []
        #         plan = []
        #         read_problem = True
        #     elif read_problem:
        #         problem.append(l)
        #     else:
        #         plan.append(l)

    PDDL_REXP = re.compile("\((.*)\)")
    def extract_action(line):
        action, status = line.split(",")
        status = plans.ActionStatusEnum.__dict__[status.strip()]
        return PDDL_REXP.search(action).group(1).lower().strip(), status

    # log.info("Loading history: %s.", history_fn)
    _task = None
    init_state = None
    
    for prob_str, plan_str in read_file(history_fn):
        t0 = time.time()
        problem = pddl.parser.Parser.parse_as(prob_str, pddl.Problem, domain)
        print "parsing problem: %.2f" % (time.time() - t0)
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
        for l in plan_str:
            action, status = extract_action(l)
            actions_status[action] = status
            if action == "init" or action == "goal":
                continue
            actions.append(action)

        if actions:
            plan = plan_postprocess.make_po_plan(list(enumerate(actions)), _task)
            for n in plan.nodes_iter():
                key = "%s %s" % (n.action.name, " ".join(a.name for a in n.full_args))
                n.status = actions_status[key.strip()]
        else:
            plan = None
        print "plan post-processing: %.2f" % (time.time() - t0)
        yield plan, state


