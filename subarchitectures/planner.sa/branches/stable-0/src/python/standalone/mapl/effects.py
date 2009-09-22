#! /usr/bin/env python
# -*- coding: latin-1 -*-

import conditions
import types

def cartesian_product(*sequences):
  # TODO: Also exists in tools.py outside the pddl package (defined slightly
  #       differently). Not good. Need proper import paths.
  if not sequences:
    yield ()
  else:
    for tup in cartesian_product(*sequences[1:]):
      for item in sequences[0]:
        yield (item,) + tup

def parse_effects(alist, result):
  """Parse a PDDL effect (simple, negative, conditional, universal conditional).
  Only understands effects with a restricted syntax:
    <effect> ::= {forall <parameters>}* <conditional-effect>
    <conditional-effect> ::= {when <condition>}? <simple-effect>
    <simple-effect> ::= {not}? <fact>"""

  orig_alist = alist # HACK!

  parameters = []
  while alist[0] == "forall":
    assert len(alist) == 3
    parameters += types.parse_typed_list(alist[1])
    alist = alist[2]

  if alist[0] == "when":
    assert len(alist) == 3
    condition = conditions.parse_condition(alist[1])
    alist = alist[2]
  else:
    condition = conditions.Truth()

  if alist[0] == "and": # This branch is a HACK!
    if len(alist) > 1:
      literal = conditions.parse_literal(alist[-1])
      effect = Effect(parameters, condition, literal)
      result.append(effect)
      alist[:] = alist[:-1]
      parse_effects(orig_alist, result)
  else:
    literal = conditions.parse_literal(alist)
    effect = Effect(parameters, condition, literal)
    result.append(effect)
  
class Effect(object):
  def __init__(self, parameters, condition, literal):
    self.parameters = parameters
    self.condition = condition
    self.literal = literal
  def copy(self):
    ps = list(self.parameters)
    c = self.condition.copy()
    l = self.literal.copy()
    return self.__class__(ps, c, l)
  def dump(self):
    indent = "  "
    if self.parameters:
      print "%sforall %s" % (indent, ", ".join(map(str, self.parameters)))
      indent += "  "
    if self.condition != conditions.Truth():
      print "%sif" % indent
      self.condition.dump(indent + "  ")
      print "%sthen" % indent
      indent += "  "
    print "%s%s" % (indent, self.literal)
  def dump_pddl(self, stream=None, indent="  "):
    if self.parameters:
      print >> stream, "%s(forall (%s)" % (indent, " ".join(map(types.pddl_str, self.parameters)))
      all_indent = indent
      indent += "  "
    if self.condition != conditions.Truth():
      print >> stream, "%s(when" % indent
      self.condition.dump_pddl(stream, indent + "  ")
      #print >> stream, "%s)" % indent
      indent += "  "
    print >> stream, "%s%s" % (indent, self.literal._dump_pddl(use_direct_k=True))
    if self.condition != conditions.Truth():
      print >> stream, "%s)" % (indent)
    if self.parameters:
      print >> stream, "%s)" % (all_indent)

  def pddl_str(self, indent="  "):
    def gen(indent):
      if self.parameters:
        yield "%s(forall (%s)" % (indent, " ".join(map(types.pddl_str, self.parameters)))
        all_indent = indent
        indent += "  "
      if self.condition != conditions.Truth():
        yield "%s(when" % indent
        yield self.condition.pddl_str(indent + "  ")
        #yield "%s)" % indent
        indent += "  "
      yield "%s%s" % (indent, self.literal._dump_pddl(use_direct_k=True))
      if self.condition != conditions.Truth():
        yield "%s)" % (indent)
      if self.parameters:
        yield "%s)" % (all_indent)
    return "\n".join(gen(indent))

  def uniquify_variables(self, type_map):
    renamings = {}
    self.parameters = [par.uniquify_name(type_map, renamings)
                       for par in self.parameters]
    self.condition = self.condition.uniquify_variables(type_map, renamings)
    self.literal = self.literal.rename_variables(renamings)
  def instantiate(self, var_mapping, init_facts, fluent_facts,
                  objects_by_type, result):
    if self.parameters:
      var_mapping = var_mapping.copy() # Will modify this.
      object_lists = [objects_by_type.get(par.type, [])
                      for par in self.parameters]
      for object_tuple in cartesian_product(*object_lists):
        for (par, obj) in zip(self.parameters, object_tuple):
          var_mapping[par.name] = obj
        self._instantiate(var_mapping, init_facts, fluent_facts, result)
    else:
      self._instantiate(var_mapping, init_facts, fluent_facts, result)
  def _instantiate(self, var_mapping, init_facts, fluent_facts, result):
    condition = []
    try:
      self.condition.instantiate(var_mapping, init_facts, fluent_facts, condition)
    except conditions.Impossible:
      return
    effects = []
    self.literal.instantiate(var_mapping, init_facts, fluent_facts, effects)
    assert len(effects) <= 1
    if effects:
      result.append((condition, effects[0]))
  def relaxed(self):
    if self.literal.negated:
      return None
    else:
      return Effect(self.parameters, self.condition.relaxed(), self.literal)
  def simplified(self):
    return Effect(self.parameters, self.condition.simplified(), self.literal)
