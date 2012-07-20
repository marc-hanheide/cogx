#!/usr/bin/env python
import sys
from standalone import pddl, task

assert len(sys.argv) == 3, """Usage: net_to_tree.py domain.pddl problem.pddl"""
domain_fn, problem_fn = sys.argv[1:]

dom = pddl.load_domain(domain_fn)
try:
    prob = pddl.load_problem(problem_fn, dom)
except:
    scenario = pddl.load_scenario(problem_fn, dom)
    prob = scenario.world

from standalone import dt_problem
pddl.dtpddl.DT2MAPLCompiler().translate(dom)
dt_rules = pddl.translators.Translator.get_annotations(dom).get('dt_rules', [])
        
prob_state = pddl.prob_state.ProbabilisticState.from_problem(prob)
objects = dom.constants | prob.objects
trees = dt_problem.StateTreeNode.create_root(prob_state, objects, dt_rules)
hstate = dt_problem.HierarchicalState([], prob_state.problem)
for t in trees:
    t.create_state(hstate)
    p_facts = hstate.init_facts()
    
p2 =  pddl.Problem(prob.name, prob.objects, p_facts, None, dom)

w = task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
w.print_problem(p2)
