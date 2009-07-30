#! /usr/bin/env python
# -*- coding: latin-1 -*-
#####################################################################
# Copyright (C) 2006 Michael Brenner                                #
#####################################################################

import os, os.path
import sys

import mapl_file
import parser
import actions


usage = "usage: %s task.mapl dom.mapl [task.pddl dom.pddl]" % sys.argv[0]
pddl_dir = "examples_mapl/pddl_output"

def outfiles(*infiles):
    for f in infiles:
        bn, ext = os.path.splitext(os.path.basename(f))
        if ext == "pddl":
            sys.exit("Attention! Input %s is probably a PDDL file already!" % f)
        yield os.path.join(pddl_dir, bn + ".pddl")
        
def parse_args(argv):
    if len(argv) == 4:
        task_in, dom_in, task_out, dom_out = argv
    elif len(argv) == 2:
        task_in, dom_in = argv[:2]
        task_out, dom_out = outfiles(task_in, dom_in)
    else:
        print usage
        sys.exit(1)
    return task_in, dom_in, task_out, dom_out

def translate(task_in, dom_in, task_out, dom_out):
    task = mapl_file.open(task_in, dom_in)
    task.write_pddl_files(task_out, dom_out)
    return task

def load_mapl_domain(fn):
    return mapl_file.open(domain_filename=fn)

def load_mapl_task(fn, mapl_domain=None):
    return mapl_file.open_task_file(fn, mapl_domain)
    
def compile_pddl_domain(mapl_domain_fn, pddl_domain_fn, keep_assertions=True):
    task = mapl_file.open(domain_filename=mapl_domain_fn)
    task.write_pddl_domain(file(pddl_domain_fn, "w"), keep_assertions)

class DomainData(object):
    def __init__(self):
        self.mapl_domain = None
        self.planning_agent = None
        self.mapl_domain_fn = None
        self.compiled_domains_path = None
        self.pddl_domain_fn = None
        self.cpddl_domain_fn = None

def prepare_compiled_planning_domains(domain_fn, planning_agent):
    dd = DomainData()
    dd.mapl_domain_fn = domain_fn
    dd.planning_agent = planning_agent
    dd.mapl_domain = load_mapl_domain(domain_fn)
    dd.mapl_domain.planning_agent = planning_agent
    import config
    dd.compiled_domains_path = config.current_data_dir
    dd.pddl_domain_fn, dd.cpddl_domain_fn = dd.mapl_domain.compile_domain_to_pddl_variants(domain_fn, dd.compiled_domains_path)
    return dd
    
def create_subgoal_op(op_tmpl, goal_condition, subgoal_name, agt_name):
    from string import Template
    tmpl = Template(op_tmpl)
    subs = dict(goal_condition=goal_condition,subgoal_name=subgoal_name, agt_name=agt_name)
    s = tmpl.safe_substitute(subs).splitlines()
    nl = parser.parse_nested_list(s)
    new_action = actions.Action._parse(nl)
    return new_action

if __name__ == "__main__":
    task_in, dom_in, task_out, dom_out = parse_args(sys.argv[1:])
    print "Translating:"
    print "  %s --> %s" % (task_in, task_out)
    print "  %s --> %s" % (dom_in, dom_out)
    print "Parsing..."
    translate(task_in, dom_in, task_out, dom_out)
    print "Done! PDDL files stored in\n  %s and\n  %s" % (task_out, dom_out)

