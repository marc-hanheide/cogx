#! /usr/bin/env python
# -*- coding: latin-1 -*-

import subprocess
import os, os.path, sys
import re

import listparser
from utils import *
import config
import syscall

last_output_fn = None

def valid_advice(l):
    l = [w for w in l if w not in "follow and or of:".split()]
    if l[0] == "set":
        if l[1][0].startswith("i__"):
            return None
        return " ".join(l[1])
    advice = [valid_advice(a) for a in l[1:]]
    if l[0] == "each" and None in advice:
        return None
    result = [a for a in advice if a is not None]
    if not result:
        return None
    if len(result) > 1:
        result.insert(0, l[0])
        return result
    else:
        return result

def extract_advice(lines):
    advice = listparser.parse_nested_list(lines, error_on_additionals=False)
    # now remove all advice for changing "initially" facts
    advice = valid_advice(advice)#
##     print "advice found:",
##     listparser.pretty_print_nested_list(advice)
    return advice

def lines_between_several(lines, startstr, endstr):
    l = []
    found = False
    for line in lines:
        if line.startswith(startstr):
            found = True
        elif found and line.startswith(endstr):
            found = False
            yield l
            l = []
        elif found:
            if not line.lower().startswith("(i__"):
                l.append(line)

def strip_parens(s):
    return [l[1:-1] for l in s if l]

def compute_states(val_output):
    states = list(lines_between_several(val_output, "STATE BEGIN", "STATE END"))
    states = [strip_parens(s) for s in states]
    return states

pattern = "(.*) \((.*)\)"
rexp = re.compile(pattern)

def compute_changes(lines):
    changes = list(lines_between_several(lines, "Checking next happening", "STATE BEGIN"))
    tup_list = []
    for change_lines in changes:
        adds, dels = [], []
        for line in change_lines:
            if not line:
                continue
            match = rexp.match(line)
            if not match:
                continue
            act, eff = match.groups()
            if act == "Adding":
                adds.append(eff)
            elif act == "Deleting":
                dels.append(eff)
        tup_list.append((adds,dels))
    return tup_list
            
def analyze_output(lines, catch_errors=True):
    results = Struct(is_executable=False, reaches_goal_state=False, advice="", changes=[])
    for i, line in enumerate(lines):
        if line.startswith("Plan executed successfully"):
            results.is_executable = True
        if line.startswith("Plan valid"):
            results.reaches_goal_state = True
        if line.startswith("The goal is not satisfied"):
            results.advice = extract_advice(lines[i+1:])
    if results.is_executable:
        results.changes = compute_changes(lines)
    states = compute_states(lines)
    if not states:
        print "\nProblem in plan monitoring."
        if catch_errors:
            print "Will be ignored - instead replanning will be triggered."
        else:
            if last_call:
                print "Have a look at file", last_output_fn, "produced by the following call:\n", last_call
            else:
                o = "\n".join(lines)
                print "len:", len(o)
                print "<<<", o, ">>>"
            sys.exit()
    results.states = states
    return results

def call_validator(pddl_prob_path, pddl_dom_path, plan=None, output_fn=None, timeout=5):
    global last_output_fn, last_call
    if not plan:
        plan = ""
    # TODO: refactor so that call_planner() can use the same code
    stdin_input = None
    if not os.path.exists(plan):
        fn = syscall.get_temporary_filename(suffix="plan")
        open(fn, "w").write(plan)
        plan = fn
    call = config.val_cmd % (pddl_dom_path, pddl_prob_path, plan)
    fn_suffix = "monitor" if plan else "goal"
    timed_out, output = syscall.run(call, input=stdin_input, timeout=timeout, fn_suffix=fn_suffix)
    if output_fn:
        f = open(output_fn, 'w')
        f.write(output)
        f.close()
        last_output_fn = output_fn
        last_call = call
    else:
        last_output_fn = syscall.last_output_fn
        last_call = syscall.last_call
    assert not timed_out, "Oops - validator shouldn't time out!\nCall was: %s\noutput:\n...%s" % (timed_out, output[-1000:])
    return output

def monitor_plan(task_path, plan, pddl_dom_path):
    val_output = call_validator(task_path, pddl_dom_path, plan)
    result = analyze_output(val_output.splitlines())
    return result
