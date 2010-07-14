import os

from standalone import task, pddl, plans
from standalone.pddl import dtpddl, mapl, translators, visitors

class DTProblem(object):
    def __init__(self, plan, domain, cast_state):
        self.plan = plan
        self.domain = domain
        self.state = cast_state
        self.subplan_actions = []
        self.goals = self.create_goals(plan)
        self.dtdomain = self.create_limited_domain(domain)
        self.create_goal_actions(self.goals, self.dtdomain)
        self.problem = self.create_problem(self.state, self.dtdomain)

        dom_str, prob_str = DTPDDLOutput().write(self.problem)
        print "\n".join(dom_str)
        print "\n".join(prob_str)

    def write_dt_input(self, domain_fn, problem_fn):
        DTPDDLOutput().write(self.problem, domain_fn=domain_fn, problem_fn=problem_fn)

    def subplan_active(self, plan):
        if not self.subplan_actions:
            return False
        
        for pnode in plan.topological_sort():
            if pnode in self.subplan_actions:
                if pnode.status == plans.ActionStatusEnum.EXECUTED:
                    self.subplan_actions.remove(pnode)
                else:
                    return True
            if pnode.status != plans.ActionStatusEnum.EXECUTED:
                return False
        assert False, "DT-Subplan actions no longer in plan!"

    def create_goals(self, plan):
        observe_actions = translators.Translator.get_annotations(self.domain).get('observe_effects', [])
        print observe_actions
        if not observe_actions:
            return None

        goal_svars = set()
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            if pnode.action.name in observe_actions:
                print pnode.action.name
                print map(str, pnode.effects)
                #TODO: only add an action if the observe effect supports a later action
                for svar, val in pnode.effects:
                    if svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                        goal_svars.add(svar.nonmodal())
                self.subplan_actions.append(pnode)
                
            if pnode.action.name not in observe_actions and self.subplan_actions:
                break
            
        return goal_svars
            
    def create_goal_actions(self, goals, domain):
        for svar in goals:
            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args())
            domain.add(svar.get_args())
            
            val = pddl.Parameter("?val", svar.function.type)
            a = pddl.Action("commit-%s" % svar.function.name, [val], None, None, domain)
            
            a.precondition = pddl.LiteralCondition(dtpddl.committed, [term], a, negated=True)
            commit_effect = pddl.SimpleEffect(dtpddl.committed, [term], a)
            reward_effect = pddl.ConditionalEffect(pddl.LiteralCondition(pddl.equals, [term, val], a), \
                                                       pddl.SimpleEffect(pddl.builtin.increase, [pddl.Term(dtpddl.reward,[]), 100]))
            a.effect = pddl.ConjunctiveEffect([commit_effect, reward_effect], a)
            
            domain.actions.append(a)
        domain.name2action = None
            

    def create_limited_domain(self, dom):
        dtdomain = dom.copy()
        dtdomain.name = "dt-%s" % dom.name
        all_actions = dtdomain.actions
        dtdomain.actions = []
        dtdomain.axioms = []

        observe_preconds = self.get_observe_action_preconditions()
        for a in all_actions:
            effects = set(visitors.visit(a.effect, function_visitor, []))
            if not observe_preconds & effects:
                dtdomain.actions.append(a)

        return dtdomain

    def create_problem(self, cast_state, domain):
        opt = "maximize"
        opt_func = pddl.FunctionTerm(pddl.dtpddl.reward, [])

        if domain is None:
            domain = self.dtdomain

        facts =  [f.to_init() for f in cast_state.prob_state.iterdists()]
        objects = set(o for o in cast_state.objects if o not in domain.constants)

        problem = pddl.Problem("cogxtask", objects, facts, None, domain, opt, opt_func )
        problem.goal = pddl.Conjunction([])
        return problem
    
    def find_observation_actions(self):
        def can_observe(action):
            for observe in self.domain.observe:
                if not observe.execution:
                    return True
                for ex in observe.execution:
                    if ex.negated:
                        return ex.action != action
                    if ex.action == action:
                        return ex
                return False

        return [a for a in self.domain.actions if can_observe(a)]
            
    def get_observe_action_preconditions(self):
        all_prec = set()
        for a in self.find_observation_actions():
            all_prec |= set(visitors.visit(a.precondition, function_visitor, []))
        return all_prec

class DTPDDLOutput(task.PDDLOutput):
    def __init__(self):
        self.compiler = pddl.translators.ChainingTranslator(dtpddl.DTPDDLCompiler(), pddl.translators.ADLCompiler())
        self.writer = dtpddl.DTPDDLWriter()

@visitors.collect
def function_visitor(elem, result):
    if isinstance(elem, pddl.FunctionTerm):
        if not elem.function.builtin:
            return sum(result, []) + [elem.function]
    if isinstance(elem, pddl.Literal):
        result = []
        if not elem.predicate.builtin:
            result = [elem.predicate]
        else:
            return result + sum([t.visit(function_visitor) for t in elem.args], [])
        return result
