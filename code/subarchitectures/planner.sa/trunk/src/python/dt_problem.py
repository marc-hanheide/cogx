import os

from collections import defaultdict
from standalone import task, pddl, plans
from standalone.pddl import state, dtpddl, mapl, translators, visitors

class DTProblem(object):
    def __init__(self, plan, domain, cast_state):
        self.plan = plan
        self.domain = domain
        self.state = cast_state
        self.subplan_actions = []
        self.select_actions = []
        
        self.goals = self.create_goals(plan)
        self.goal_actions = set()
        self.compute_restrictions()
        
        self.dtdomain = self.create_limited_domain(domain)
        self.goal_actions |= self.create_goal_actions(self.goals, self.dtdomain)
        self.dtdomain.actions += [a for a in self.goal_actions]
        self.dtdomain.name2action = None
        
        self.problem = self.create_problem(self.state, self.dtdomain)
        self.dt_plan = []

        # dom_str, prob_str = DTPDDLOutput().write(self.problem)
        # print "\n".join(dom_str)
        # print "\n".join(prob_str)

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
        #print observe_actions
        if not observe_actions:
            return []

        def find_restrictions(pnode):
            for pred in plan.predecessors_iter(pnode, 'depends'):
                restr = find_restrictions(pred)
                if pred.action.name.startswith("select_"):
                    return [pred] + restr
                if restr:
                    return restr
            return []

        goal_svars = set()
        #combine consecutive observe actions into one subtask.
        for pnode in plan.topological_sort():
            if pnode.status == plans.ActionStatusEnum.EXECUTED:
                continue
            if pnode.action.name in observe_actions:
                #print pnode.action.name
                #print map(str, pnode.effects)
                #TODO: only add an action if the observe effect supports a later action
                for svar, val in pnode.effects:
                    if svar.modality in (mapl.knowledge, mapl.direct_knowledge):
                        goal_svars.add(svar.nonmodal())
                self.subplan_actions.append(pnode)
                self.select_actions = [pnode] + find_restrictions(pnode)
                break # only one action at a time for now
            
            if pnode.action.name not in observe_actions and self.subplan_actions:
                break
            
        return goal_svars

    def compute_restrictions(self):
        layers = []
        relaxations = set()
        fixed = set()
        constraints = defaultdict(set)
        for pnode in self.select_actions:
            print "action:",pnode
            print "cond:",map(str,pnode.preconds)
            new_fixed = fixed | set(pnode.full_args)
            new_relaxations = set()
            new_constraints = defaultdict(set)
            for svar, val in pnode.preconds:
                if svar.modality == mapl.commit:
                    new_relaxations |= set(svar.args+svar.modal_args)
                    new_relaxations.add(val)
                elif svar.function.type != pddl.t_number:
                    for a in svar.args+svar.modal_args:
                        new_constraints[a].add(state.Fact(svar,val))
            new_relaxations.discard(pddl.TRUE)
            new_relaxations.discard(pddl.FALSE)
            
            for svar in relaxations:
                print "trying to relax", svar,"..."
                if svar in new_relaxations:
                    print "will be relaxed later"
                    continue
                if svar in new_constraints:
                    print "has some new constraints:", map(str, constraints[svar])
                    new_fixed.discard(svar)
                    continue
                print "is completely free"
                new_fixed.discard(svar)

            for obj, constr in constraints.iteritems():
                for c in constr:
                    if c.value in new_fixed:
                        new_constraints[obj].add(c)
                    else:
                        print obj.name, "constraint", c, "is gone"
                        
            
            print "fixed on this layer:", map(str, new_fixed)
            c2 = []
            replace_dict = {}
            for obj, cset in new_constraints.iteritems():
                for c in cset:
                    if obj in replace_dict or obj in new_fixed:
                        continue
                    args = []
                    assert not c.svar.modal_args, "Not yet supported."
                    for a in c.svar.args:
                        a2 = pddl.TypedObject("any_%s" % (a.type.name), a.type)
                        replace_dict[a] = a2
                        args.append(a2)
                    val = replace_dict.get(c.value, c.value)
                    f2 = pddl.state.Fact(pddl.state.StateVariable(c.svar.function, args), val)
                    if f2 not in c2:
                        c2.append(f2)
                
            layers.append((new_fixed, c2))
            #print "fixed values:", map(str, fixed)
            #print "possible relaxations:", map(str, relaxations)
            fixed = new_fixed
            relaxations = new_relaxations
            constraints = new_constraints

        for i,(fixed,constraints) in enumerate(layers):
            print "Layer",i
            print "Fixed:", map(str, fixed)
            # cset = set()
            # for c in constraints.itervalues():
            #     cset |= c
            print "Constraints:", map(str, constraints)
            print "\n"
            
    def create_goal_actions(self, goals, domain):
        result = set()
        for svar in goals:
            term = pddl.Term(svar.function, svar.get_args())
            domain.constants |= set(svar.get_args())
            domain.add(svar.get_args())
            
            val = pddl.Parameter("?val", svar.function.type)
            name = "commit-%s-%s" % (svar.function.name, "-".join(a.name for a in svar.get_args()))
            a = pddl.Action(name, [val], None, None, domain)
            
            a.precondition = pddl.LiteralCondition(dtpddl.committed, [term], a, negated=True)
            commit_effect = pddl.SimpleEffect(dtpddl.committed, [term], a)
            reward_effect = pddl.ConditionalEffect(pddl.LiteralCondition(pddl.equals, [term, val], a), \
                                                       pddl.SimpleEffect(pddl.builtin.increase, [pddl.Term(dtpddl.reward,[]), 100]))
            a.effect = pddl.ConjunctiveEffect([commit_effect, reward_effect], a)
            
            result.add(a)
        return result

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
