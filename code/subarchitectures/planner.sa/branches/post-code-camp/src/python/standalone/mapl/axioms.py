#! /usr/bin/env python
# -*- coding: latin-1 -*-

import conditions
import predicates
import types

class Axiom(object):
  def __init__(self, name, parameters, condition):
    self.name = name
    self.parameters = parameters
    self.condition = condition
    #self.uniquify_variables()
    
  def parse(alist):
    assert len(alist) == 3
    assert alist[0] == ":derived"
    predicate = predicates.Predicate.parse(alist[1])
    condition = conditions.parse_condition(alist[2])
    ax = Axiom(predicate, predicate.arguments, condition)
    return ax
  parse = staticmethod(parse)
  
  def dump(self):
    print "Axiom %s(%s)" % (self.name, ", ".join(map(str, self.parameters)))
    self.condition.dump()

  def dump_pddl(self, stream):
    print >> stream, "\n".join(self.pddl_gen())

  def pddl_gen(self):
    pred = self.name
    assert isinstance(pred, predicates.Predicate)
    yield "\n(:derived (%s)" % (pred.pddl_str())
    condition = self.condition
    if pred.is_boolean():
      condition = condition.copy()
    yield condition.pddl_str(indent="    ")
    yield ")"

  def uniquify_variables(self):
    self.type_map = dict([(par.name, par.type) for par in self.parameters])
    self.condition = self.condition.uniquify_variables(self.type_map)
    
  def instantiate(self, var_mapping, init_facts, fluent_facts):
    # The comments for Action.instantiate apply accordingly.
    arg_list = [var_mapping[par.name] for par in self.parameters]
    name = "(%s %s)" % (self.name, " ".join(arg_list))

    condition = []
    try:
      self.condition.instantiate(var_mapping, init_facts, fluent_facts, condition)
    except conditions.Impossible:
      return None

    effect_args = [var_mapping.get(arg.name, arg.name) for arg in self.parameters]
    effect = conditions.Atom(self.name, effect_args)
    return PropositionalAxiom(name, condition, effect)

class PropositionalAxiom:
  def __init__(self, name, condition, effect):
    self.name = name
    self.condition = condition
    self.effect = effect
  def clone(self):
    return PropositionalAxiom(self.name, list(self.condition), self.effect)
  def dump(self):
    if self.effect.negated:
      print "not",
    print self.name
    for fact in self.condition:
      print "PRE: %s" % fact
    print "EFF: %s" % self.effect
