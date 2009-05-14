#! /usr/bin/env python
# -*- coding: latin-1 -*-

import conditions
import sys
import utils

class Type(object):
  def __init__(self, astring):
    if astring[0] == "either":
      self.name = "(%s)" % " ".join(astring)
    else:
      self.name = astring
  def __str__(self):
    return self.name

class TypedObject(object):
  def __init__(self, name, type):
    self.name = name
    if not utils.is_string(type):
      type = tuple(type)
    self.type = type
  def __hash__(self):
    return hash((self.name, self.type))
  def __eq__(self, other):
    try:
      return self.name == other.name and self.type == other.type
    except: return False
  def __ne__(self, other):
    return not self == other
  def __str__(self):
    return "%s: %s" % (self.name, self.type)
  def uniquify_name(self, type_map, renamings):
    if self.name not in type_map:
      type_map[self.name] = self.type
      return self
    for counter in xrange(1, sys.maxint):
      new_name = self.name + str(counter)
      if new_name not in type_map:
        renamings[self.name] = new_name
        type_map[new_name] = self.type
        return TypedObject(new_name, self.type)
  def to_untyped_strips(self):
    return conditions.Atom(self.type, [self.name])
  def to_object_declaration(self):
    type_str = str(self.type)
    return "%s - %s" % (self.name, type_str)
    

def typed_list2dict(alist):
  d = {}
  if len(alist) < 2 or alist[-2] != "-":
    alist += ["-", "object"]
  rlist = list(reversed(alist))
  last_elmt = None
  for elmt in rlist:
    if elmt == "-":
      sup_type = last_elmt
      last_elmt = None
    else:
      if last_elmt:
        d[last_elmt] = sup_type
      last_elmt = elmt
  if last_elmt:
    d[last_elmt] = sup_type
  return d
      
def parse_typed_list(alist, only_variables=False):
  result = []
  if "-" not in alist:
    alist += ["-", "object"]
  while alist:
    try:
      separator_position = alist.index("-")
    except ValueError:
      print alist, "does not contain -"
      raise
    names = alist[:separator_position]
    if not names:
      break
    type = alist[separator_position + 1]
    for name in names:
      assert name.startswith("?") or not only_variables, "%s makes problems" % name
      result.append(TypedObject(name, type))
    alist = alist[separator_position + 2:]
  return result

def pddl_str(obj):
  return "%s - %s" % (obj.name, obj.type)

mapl_str = pddl_str

def pddl_type_str(type):
  if type[0] == "either":
    type_str = "(%s)" % " ".join(type)
  else:
    type_str = str(type)
  return type_str

