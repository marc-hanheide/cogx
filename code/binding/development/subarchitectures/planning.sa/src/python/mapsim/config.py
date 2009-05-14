#! /usr/bin/env python
# -*- coding: latin-1 -*-

import sys, os
from os.path import dirname, abspath, join, exists, lexists

import constants
from utils import str2python_obj
from tools.path_tools import path_from_cwd

def load_configuration(fname):
    def item_gen(fname):
        for line in open(fname):
            line = line.split("#", 1)[0].strip()  # remove comments and whitespace
            if line:
               varname, value = line.split("=", 1)
               varname = str2python_obj(varname)
               value = str2python_obj(value)
               yield varname, value
    config_dict = globals()
    for varname, value in item_gen(fname):
        config_dict[varname] = value

def add_configuration(source_dict):
    config_dict = globals()
    for varname, value in source_dict.iteritems():
        config_dict[varname] = value

def get_config_diff(dict):
    current_config_dict = globals()
    diffs = {}
    for varname, value in dict.iteritems():
        if current_config_dict.get(varname) != value:
            diffs[varname] = current_config_dict.get(varname)

    return diffs

config_filename = "CONFIG"  # is automatically extended with path later

##########################

# globals for internal use. don't touch these

absolute_path = dirname(abspath(__file__))
mapsim_dir = absolute_path
config_filename = path_from_cwd(config_filename, mapsim_dir)
planning_dir = os.path.join(mapsim_dir, "planning")
if not exists(planning_dir):
#     print "map", mapsim_dir
#     print "pla", planning_dir
#     print "conf", config_filename
    planning_dir = absolute_path
    mapsim_dir = path_from_cwd("..", absolute_path)  # TODO: not nice
    assert exists(path_from_cwd("planner_base", planning_dir))
planner_call = path_from_cwd("planner_base/ContinualAxff/ff %s-o %s -f %s", planning_dir)
val_cmd = path_from_cwd("plan_monitor/VAL/validate -v %s %s %s", planning_dir)
common_data_path = path_from_cwd("data", mapsim_dir)
current_data_dir = path_from_cwd("current_data", mapsim_dir, use_abs_path=True)
current_run_dir = path_from_cwd("current_run", mapsim_dir, use_abs_path=True)
domains_path = "domains"

# computed values. don't touch these here
domain_module = None
runs_name = None
domain_name = None
suite_name = None
prob_name = None
unique_scenario_name = None  
setting_str = None
current_agent = None
simulation = None
time = 0
mapl_domain = None

# logging and statistics
log_fn = None
stats_fn = None
log_file = None
stats_file = None
log2screen = True
log2file = True

statistics = {}   # every agent, simulator, other component can store their stats here
all_statistics = {}  # stats for every scenario in the test suite currently run

# timeout, durations, etc.
INDIV_TIMEOUT = 10       # allow each agent INDIV_TIMEOUT secs for planning
REL_DURATION_ROUND = 1   # duration = #agents * INDIV_TIMEOUT * REL_DURATION_ROUND


# templates for 2nd-order operator descriptions

explicit_value_known_tmpl = ("""
;; multi-valued version
(:derived (k ?_a - agent ($svar $args_decl))
     (exists (?_v - $domtype) (and 
        (is_planning_agent ?_a)
        ($svar $args : ?_v))))
""", """
;; boolean version
(:derived (k ?_a - agent ($svar $args_decl))
     (and
        (is_planning_agent ?_a)
        (or ($svar $args) (not ($svar $args)))))
""")

knowledge_directly_asserted = ("""
;; multi-valued version
(:derived (k ?_a - agent ($svar $args_decl))
        (kd__$svar ?_a $args))
""", """
;; boolean version
(:derived (k ?_a - agent ($svar $args_decl))
        (kd__$svar ?_a $args))
""")


# default list of templates to use
op_templates_2nd_order = [
    knowledge_directly_asserted, 
    explicit_value_known_tmpl
]


subgoal_op = """
(:action __achieve_subgoal_$subgoal_name
 :agent (?a - planning_agent)
 :precondition $precondition
 :effect (and
	(achieved $subgoal_name))
)
"""

top_goal_for_subgoals = """(forall (?sg - subgoal) (achieved ?sg))"""


# helper functions

def link(path, link_path):
    path = os.path.abspath(path)
    link_path = os.path.abspath(link_path)
    if lexists(link_path):
        os.unlink(link_path)
#     print "linking %s to %s" % (path, link_path)
    os.symlink(path, link_path)
    
def setup_log_and_stats(scenario_name=None):
    global log_fn, log_file, stats_fn, stats_file, unique_scenario_name
    unique_scenario_name = scenario_name
    if unique_scenario_name is None:
        usn = "%s.%s.%s.%s" % (domain_name, suite_name, prob_name, setting_str)
        unique_scenario_name = usn.replace("/", ".") 
    log_fn = "%s.log" % unique_scenario_name
    stats_fn = "%s.stat" % unique_scenario_name
    log_path = path_from_cwd(log_fn, current_data_dir)
    stats_path = path_from_cwd(stats_fn, current_data_dir)
    log_file = open(log_path, "w")
    stats_file = open(stats_path, "w")
    log_link = path_from_cwd("current_log", current_data_dir)
    link(log_path, log_link)
    stats_link = path_from_cwd("current_stats", current_data_dir)
    link(stats_path, stats_link)
    import syscall
    syscall.setup_stats()

def split_linebreaks(msg):
    msg_nolinebreaks = msg.lstrip("\n")
    linebreaks = msg[0:len(msg)-len(msg_nolinebreaks)]
    return msg_nolinebreaks, linebreaks

def get_mapl_domain():
    if mapl_domain is not None:
        return mapl_domain
    return simulation.mapl_domain

default_log_verbosity = 100
default_info_mode = 9999


def log(*msg, **params):
    # usage: log(msg, screen=True, file=False, vlevel=17, info=8)
    param_names = "file screen vlevel info agent".split()
    defaults = [log2file, log2screen, default_log_verbosity, default_info_mode, current_agent]
    param = dict(zip(param_names, defaults))
    param.update((n,params[n]) for n in params)
    tofile, toscreen, vlevel, info_mode, agt_name = [param[n] for n in param_names]
    toscreen = toscreen and (vlevel <= verbosity or info_mode == info)
    tofile = tofile and (vlevel <= verbosity_file or info_mode == info)
    if (not tofile) and (not toscreen):
        return
    if isinstance(msg, tuple):
        msg = " ".join(map(str,msg))
    msg, linebreaks = split_linebreaks(msg)
    if agt_name:
        msg = "(%s) %s" % (agt_name, msg)
    msg = linebreaks + msg
    if tofile and isinstance(log_file,file):
        print >>log_file, msg
        log_file.flush()
    if toscreen:
        print msg
        sys.stdout.flush()

# finally, load the configuration file
load_configuration(config_filename)
