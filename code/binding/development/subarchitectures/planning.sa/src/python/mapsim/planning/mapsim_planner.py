#!/usr/bin/env python

import os, os.path, sys

try:
    import utils
except ImportError:
    mapsim_main_dir = os.path.join(sys.path[0],"..")
    sys.path.append(mapsim_main_dir)
    print sys.path
    import utils

import plans
from plans import POPlan
import mapl, planner_base, eng2mapl, plan_monitor, memory
from config import log


ABSOLUTE_PATH = os.path.dirname(os.path.abspath(sys.argv[0]))
TMP_DIR = "tmp"
FTYPES = "task domain plan".split()
tmp_count = 0

task_template =  """(define (problem %s)\n(:domain %s)\n""" \
                """(:objects\n%s\n)\n(:init\n%s\n)\n\n(:goal \n%s\n))"""
DEFAULT_DOM  = "cosydomain"
DEFAULT_TASK = "cosytask"

DEBUG = False 
VERBOSE = False


def val_format(plan):
    l = ["%s: (%s)" % (i, action) for i, action in enumerate(plan)]
    return "\n".join(l)

PM_ID = "pm1"

class MapsimPlanner(object):
    def __init__(self):
        self.PMs = {PM_ID : memory.PlanningMemory()}
    
    def monitor_plan(self, plan, task, pddl_dom_path, output_path=None):
        if isinstance(plan, POPlan):
            plan = plan.total_order()
        assert utils.is_seq_but_not_string(plan) or (utils.is_string(plan) and os.path.exists(plan))
        if utils.is_seq_but_not_string(plan):
            plan = val_format(plan)
        result = plan_monitor.monitor_plan(task, plan, pddl_dom_path)
        return result

    def find_plan(self, task, domain_fn, output_path=None):
        """returns a POPLAN"""
        plan, action_descriptions = planner_base.find_plan(task, domain_fn, output_path)
        return plan, action_descriptions

