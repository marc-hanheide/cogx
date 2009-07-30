#! /usr/bin/env python
# -*- coding: latin-1 -*-

import sys
import os.path
import re

import parser

import tasks

def parse_mapl_file(type, filename):
  try:
    it = file(filename)
  except (TypeError,IOError):
    it = filename
  try:
    return parser.parse_nested_list(it)
  except IOError, e:
    raise SystemExit("Error: Could not read file: %s\nReason: %s." %
                     (e.filename, e))
  except parser.ParseError, e:
    raise SystemExit("Error: Could not parse %s file: %s\nReason: %s" % (type, filename, e))

# def open(task_filename=None, domain_filename=None):
#   assert domain_filename is not None     # TODO: change param order
#   domain_mapl = parse_mapl_file("domain", domain_filename)
#   if task_filename:
#     task_mapl = parse_mapl_file("task", task_filename)
#   else:
#     task_mapl = None
#   return tasks.Task.parse(domain_mapl, task_mapl)

# def open_task_file(task_filename, mapl_domain):
#   task_mapl = parse_mapl_file("task", task_filename)
#   return tasks.parse_task(task_mapl, mapl_domain)

def load_mapl_domain(domain_filename):
  domain_mapl = parse_mapl_file("domain", domain_filename)
  return tasks.Task.parse(domain_mapl)

def load_mapl_problem(task_filename):
  task_mapl = parse_mapl_file("task", task_filename)
  return tasks.parse_task(task_mapl)
    


if __name__ == "__main__":
  open().dump()
