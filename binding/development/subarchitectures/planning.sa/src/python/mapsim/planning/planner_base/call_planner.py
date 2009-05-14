import sys, os, os.path
import re

import syscall
import util
import config
import plans
from constants import *

IGNORED_ACTIONS = "reach-goal".split()
KD_PREFIX = "(kd__"
K_PREFIX = "(k__"
NEG_PREFIX = "(not-"
NPLEN = len(NEG_PREFIX)

def ignore_action(a):
    return a in IGNORED_ACTIONS

def remove_kd(eff):
    if eff.startswith(KD_PREFIX):
        return eff.replace(KD_PREFIX, K_PREFIX)
    return eff

def find_long_actions(lines, num):
    mapl_domain = config.get_mapl_domain()
    for i in range(num):
        #print "reading action %d of %d" % (i, num)
        unconditional_effect = False
        adds = []
        dels = []
        while True:
            line = lines.nextline().lower()
            #print line
            if line.startswith("action"):
                action = " ".join(line.split()[1:])
            elif line.startswith("preconds"):
                n = int(lines.nextline())
                preconds = lines.next(n)
            elif line.startswith("conditions"):
                n = int(lines.nextline())
                unconditional_effect = True #(n == 0)
            elif line.startswith("adds") and unconditional_effect:
                n = int(lines.nextline())
                adds.extend(lines.next(n))
                #print "adds of action found"
            elif line.startswith("dels") and unconditional_effect:
                n = int(lines.nextline())
                dels.extend(lines.next(n))
                #print "dels of action found"
            elif line.startswith("end action"):
                #print "end of action found"
                break
        adds = [remove_kd(eff) for eff in adds]
        dels = [remove_kd(eff) for eff in dels]
#         print "tup1", (action, preconds, adds, dels)
        preconds, adds, dels = [dict(mapl_domain.props2svar_assignments(fl, ignore_negated_props=False)) for fl in (preconds, adds, dels)]
#         print "tup2", (action, preconds, adds, dels)
        if not ignore_action(action):
            yield (action, preconds, adds, dels)

def parse_plan(planner_output):
    step_re = r"step.*:(.*)"
    r = re.compile(step_re)
    time_re = r"(\d*) ms total time"
    time = re.compile(time_re)
    plan = long_plan = None
    lines = util.ListCursor(planner_output.splitlines())
    dur = -1
    num_actions = -1
    while True:
        if not lines.has_next():
            break
        line = lines.nextline()
        if line.find("success") >= 0:
            plan = []
            long_plan = []
            num_actions = int(lines.nextline())
            actions = lines.next(num_actions)
            for a in actions:
                match = r.search(a.lower())
                if match:
                    action = match.group(1).strip()
                    if not ignore_action(action):
                        #plan.append(action.split())
                        plan.append(action)
        if line.find("long plan") >= 0:
           long_plan = list(find_long_actions(lines, num_actions))
           break
        match = time.search(line)
        if match:
            dur = int(match.group(1))
            #print "used %d ms for planning" % dur
    return plan, long_plan


def call_planner(problem, domain, output_fn=None, timeout=5, options="", fn_suffix=None):
    # TODO: refactor so that monitor.call_validator() can use the same code
    stdin_input = None
    if not os.path.exists(problem):
        # problem is not a file path, but a string
        stdin_input = problem
        problem = "/dev/stdin"
    call = config.planner_call % (options, domain, problem)
    timed_out, output = syscall.run(call, input=stdin_input, timeout=timeout, fn_suffix=fn_suffix)
    output = output.lower()
    if output_fn:
        f = open(output_fn, 'w')
        f.write(output)
        f.close()
    return timed_out, output

    
def find_plan(task, domain_fn, output_fn=None):
    timed_out, output = call_planner(task, domain_fn, output_fn, fn_suffix="planning")
    #assert not timed_out, "Oops - planner shouldn't time out for simple probs\nCall was: %s\noutput:\n...%s" % (timed_out, output[-1000:])
    if timed_out:
        return None
    return parse_plan(output)

def full_state_and_enabled_actions(task, domain, output_path=None):
    timed_out, output = call_planner(task, domain, output_path, options="-A ", timeout=5, fn_suffix="actions")
    assert not timed_out, "Oops - planner shouldn't time out when computing applicable actions...\nCall was: %s\noutput:\n...%s" % (timed_out, output[-1000:])
    lines = output.splitlines()
    actions, facts, assertions = [], [], []
    d = dict(action=actions, fact=facts, assertion=assertions)
    marker = "!!!"
    for line in lines:
        if not line[0:3] == marker:
            continue
        linemarker, kind, info = line.split(None, 2)
        d[kind].append(info.strip())
    return facts, actions, assertions

def applicable_actions(task, domain, output_path=None):
    state, ops, assertions = full_state_and_enabled_actions(task, domain, output_path)
    return ops

