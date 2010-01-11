#! /usr/bin/env python
# -*- coding: latin-1 -*-

from constants import *
import types
import actions
import conditions
import effects

default_agents = [types.TypedObject("?agt%d" %i, "agent") for i in (0,1)]

class Predicate(object):
  def __init__(self, name, arguments, value, believers=None, initially=False):
    self.name = name
    self.believers = believers
    self.arguments = arguments
    self.initially = initially
    if value is None:
      value = [types.TypedObject("?_val", "boolean")]
    self.value = value
    # various string representations of parts of the predicate
    self.args_decl_str = " ".join("%s - %s" % (d.name, d.type) for d in self.arguments)
    self.args_str = " ".join(d.name for d in self.arguments)
    self.value_type_str = " ".join(types.pddl_type_str(d.type) for d in self.value)
    #print "vts:", self.value_type_str

  @staticmethod
  def parse(alist):
    name = alist[0]
    if name == K_PREFIX or name == DIRECT_K_PREFIX:
      return ModalPredicate.parse(alist)
    if ":" not in alist:
      alist.extend(": ?_val - boolean".split())
    separator_position = alist.index(":")
    arguments = types.parse_typed_list(alist[1:separator_position], only_variables=True)
    value = types.parse_typed_list(alist[separator_position+1:], only_variables=True)
    result = Predicate(name, arguments, value)
    return result

  @staticmethod
  def svar_in_domain_predicate_name(svar_name):
    return "%s%s%s" % (IN_DOMAIN_KW, PREFIX_SEP, svar_name)

  def svar_in_domain_predicate(self):
    name = Predicate.svar_in_domain_predicate_name(self.name)
    return Predicate(name, self.arguments, value=self.value, believers=self.believers)

  @staticmethod
  def svar_domain_predicate_name(svar_name):
    return "%s%s%s" % (DOMAIN_KW, PREFIX_SEP, svar_name)

  def svar_domain_predicate(self):
    name = Predicate.svar_domain_predicate_name(self.name)
    return Predicate(name, self.arguments, value=self.value, believers=self.believers)

  def arity(self):
    return len(self.arguments)

  def is_boolean(self):
    if self.value[0].type == "boolean":
      return True
    if bool(self.believers):
      return True
#     if self.name.startswith(IN_DOMAIN_KW):
#       return True
   
  def is_multi_valued(self):
    return not self.is_boolean()
 
  def initially_pred(self):
    return Predicate(self.name, self.arguments, value=self.value, believers=self.believers,initially=True)    

  def knowledge_pred(self, believers):
    if self.believers:
      # no beliefs about beliefs - for the time being
      return None
    return Predicate(self.name, self.arguments, value=None, believers=believers)
  
  def __str__(self):
    if self.is_boolean():
      s = "%s(%s)" % (self.name, ", ".join(map(str, self.arguments)))
    else:
      s = "%s(%s : %s)" % (self.name, ", ".join(map(str, self.arguments)), ", ".join(map(str, self.value)))
    if self.believers:
      if len(self.believers) == 1:
        raise Exception("something's wrong with", self.predicate, self.believers)
      else:
        if self.believers[0].name == self.believers[1].name:
          s = "k(%s %s)" % (" ".join(map(str, self.believers[0])), s)
        else:
          s = "mb(%s %s)" % (" ".join(map(str, self.believers)), s)        
    return s

  def pddl_str_name(self, k_axiom=False):
    name = self.name
    if self.believers:
      if k_axiom:
        name = "kval__%s" % name
      else:
        name = "kvald__%s" % name
    if self.initially:
      name = INITIALLY_PREFIX + PREFIX_SEP + name
    return name

  def pddl_str(self, k_axiom=False):
    name = self.pddl_str_name(k_axiom)
    args = self.arguments
    if self.believers:
      args = self.believers + args
#       return "%s %s" % (name, " ".join(map(types.pddl_str, args)))
    return "%s %s %s" % (name, " ".join(map(types.pddl_str, args)), " ".join(map(types.pddl_str, self.value)))
    


class ModalPredicate(Predicate):
  def __init__(self, modal_op, arguments, predicate):
    Predicate.__init__(self, modal_op, arguments, None)
    self.predicate = predicate

  @staticmethod
  def parse(alist):
    predicate = Predicate.parse(alist[-1])
    name = alist[0]
    args = alist[1:-1]
    arguments = types.parse_typed_list(args, only_variables=True)
    return ModalPredicate(name, arguments, predicate)

  def is_boolean(self):
    # this is not necessarily true forever, but right now, we
    # do only allow positive occurences
    return True

  def pddl_str(self):
    name = self.name
    args = self.arguments
    name = "%s__%s" % (name, self.predicate.name)
    args = args + self.predicate.arguments + self.predicate.value
    return "%s %s" % (name, " ".join(map(types.pddl_str, args)))
    
    

