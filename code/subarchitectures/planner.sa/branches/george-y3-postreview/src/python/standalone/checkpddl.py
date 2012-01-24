#!/usr/bin/env python
import sys, pddl

assert len(sys.argv) > 1, """Call 'planner.py domain.pddl [problem.pddl]' for a single planner call"""
if len(sys.argv) == 3:
    domain_fn, problem_fn = sys.argv[1:]
else:
    problem_fn = None
    domain_fn = sys.argv[1]

print "Loading domain...",
dom = pddl.load_domain(domain_fn)
print "OK"

if problem_fn:
    print "Loading problem...",
    prob = pddl.load_problem(problem_fn, dom)
    print "OK"
