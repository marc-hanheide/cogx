#! /usr/bin/env python
# -*- coding: latin-1 -*-

import sys
import sets
import itertools

import config
from constants import *
import state
import types
from utils import codepos

def parse_condition(alist):
  condition = parse_condition_aux(alist, False)
  condition.uniquify_variables({})
  return condition

def parse_condition_aux(alist, negated):
  """Parse a PDDL condition. The condition is translated into NNF on the fly."""
  tag = alist[0]
  if tag in ("and", "or", "not", "imply"):
    args = alist[1:]
    if tag == "imply":
      assert len(args) == 2
    if tag == "not":
      assert len(args) == 1
      return parse_condition_aux(args[0], not negated)
  elif tag in ("forall", "exists"):
    parameters = types.parse_typed_list(alist[1])
    args = alist[2:]
    assert len(args) == 1
  elif negated:
    #return NegatedAtom(alist[0], alist[1:])
    #return Atom(alist[0], alist[1:], value=FALSE_TUP)
    return parse_literal(alist, negated=True)
  else:
    #return Atom(alist[0], alist[1:])
    return parse_literal(alist)
  if tag == "imply":
    parts = [parse_condition_aux(args[0], not negated),
             parse_condition_aux(args[1], negated)]
    tag = "or"
  else:
    parts = [parse_condition_aux(part, negated) for part in args]

  if tag == "and" and not negated or tag == "or" and negated:
    return Conjunction(parts)
  elif tag == "or" and not negated or tag == "and" and negated:
    return Disjunction(parts)
  elif tag == "forall" and not negated or tag == "exists" and negated:
    return UniversalCondition(parameters, parts)
  elif tag == "exists" and not negated or tag == "forall" and negated:
    return ExistentialCondition(parameters, parts)




# Conditions (of any type) are immutable, because they need to
# be hashed occasionally. Immutability also allows more efficient comparison
# based on a precomputed hash value.
#
# Careful: Most other classes (e.g. Effects, Axioms, Actions) are not!

class Condition(object):
  def __init__(self, parts):
    self.parts = tuple(parts)
    self.hash = hash((self.__class__, self.parts))
  def __hash__(self):
    return self.hash
  def __eq__(self, other):
    # Compare hash first for speed reasons.
    return (self.hash == other.hash and
            self.__class__ is other.__class__ and
            self.parts == other.parts)
  def __ne__(self, other):
    return not self == other
  def copy(self):
    parts = [p.copy() for p in self.parts]
    try:
      return self.__class__(parts)
    except:
      print "class", self.__class__
      raise
  def dump(self, indent="  "):
    print "%s%s" % (indent, self._dump())
    for part in self.parts:
      part.dump(indent + "  ")
  def _dump(self):
    return self.__class__.__name__
  def dump_pddl(self, stream=None, indent=" "):
    print >> stream, self.pddl_str(indent)

  def as_str(self, method_name, indent="" ):
    method = getattr(self, method_name)
    l = ["%s%s" % (indent, method())]
    for part in self.parts:
      l.append(part.as_str(method_name, indent + "  "))
    # bad HACK following
    if not isinstance(self, Literal):
      l.append("%s)" % indent)
    return "\n".join(l)

  def pddl_str(self, indent=""):
    return self.as_str("_dump_pddl", indent)
  def mapl_str(self, indent=""):
    return self.as_str("_dump_mapl", indent)
  def _dump_mapl(self):
    return self._dump_pddl()
  def _postorder_visit(self, method_name, *args):
    part_results = [part._postorder_visit(method_name, *args)
                    for part in self.parts] 
    method = getattr(self, method_name, self._propagate)
    return method(part_results, *args)
  def _propagate(self, parts, *args):
    return self.change_parts(parts)
  def simplified(self):
    return self._postorder_visit("_simplified")
  def relaxed(self):
    return self._postorder_visit("_relaxed")
  def untyped(self):
    return self._postorder_visit("_untyped")

  def uniquify_variables(self, type_map, renamings={}):
    # Cannot used _postorder_visit because this requires preorder
    # for quantified effects.
    if not self.parts:
      return self
    else:
      return self.__class__([part.uniquify_variables(type_map, renamings)
                             for part in self.parts])
  def to_untyped_strips(self):
    raise ValueError("Not a STRIPS condition: %s" % self.__class__.__name__)
  def instantiate(self, var_mapping, init_facts, fluent_facts, result):
    raise ValueError("Cannot instantiate condition of type %s: not normalized" % type(self))
  def free_variables(self):
    result = sets.Set()
    for part in self.parts:
      result |= part.free_variables()
    return result
  def has_disjunction(self):
    for part in self.parts:
      if part.has_disjunction():
        return True
    return False
  def has_existential_part(self):
    for part in self.parts:
      if part.has_existential_part():
        return True
    return False
  def has_universal_part(self):
    for part in self.parts:
      if part.has_universal_part():
        return True
    return False
  def make_conjunction(self):
    if isinstance(self, Conjunction):
      return self
    else:
      return Conjunction([self])

class ConstantCondition(Condition):
  parts = ()
  def __init__(self):
    self.hash = hash(self.__class__)
  def copy(self):
    return self.__class__()
  def change_parts(self, parts):
    return self
  def __eq__(self, other):
    return self.__class__ is other.__class__
  def __ne__(self, other):
    return not self == other

class Impossible(Exception):
  pass

class Falsity(ConstantCondition):
  def instantiate(self, var_mapping, init_facts, fluent_facts, result):
    raise Impossible()
  def negate(self):
    return Truth()

class Truth(ConstantCondition):
  def to_untyped_strips(self):
    return []
  def instantiate(self, var_mapping, init_facts, fluent_facts, result):
    pass
  def negate(self):
    return Falsity()

class JunctorCondition(Condition):
  def change_parts(self, parts):
    return self.__class__(parts)

class Conjunction(JunctorCondition):
  def _simplified(self, parts):
    result_parts = []
    for part in parts:
      if isinstance(part, Conjunction):
        result_parts += part.parts
      elif isinstance(part, Falsity):
        return Falsity()
      elif not isinstance(part, Truth):
        result_parts.append(part)
    if not result_parts:
      return Truth()
    if len(result_parts) == 1:
      return result_parts[0]
    return Conjunction(result_parts)
  def to_untyped_strips(self):
    result = []
    for part in self.parts:
      result += part.to_untyped_strips()
    return result
  def instantiate(self, var_mapping, init_facts, fluent_facts, result):
    assert not result, "Condition not simplified"
    for part in self.parts:
      part.instantiate(var_mapping, init_facts, fluent_facts, result)
  def negate(self):
    return Disjunction([p.negate() for p in self.parts])
  def _dump_pddl(self):
    return "(and"
  def as_svar_assignments(self, replacements={}):
    result = []
    for cond in self.parts:
      result += cond.as_svar_assignments(replacements)
    return result

class Disjunction(JunctorCondition):
  def _simplified(self, parts):
    result_parts = []
    for part in parts:
      if isinstance(part, Disjunction):
        result_parts += part.parts
      elif isinstance(part, Truth):
        return Truth()
      elif not isinstance(part, Falsity):
        result_parts.append(part)
    if not result_parts:
      return Falsity()
    if len(result_parts) == 1:
      return result_parts[0]
    return Disjunction(result_parts)
  def negate(self):
    return Conjunction([p.negate() for p in self.parts])
  def has_disjunction(self):
    return True
  def _dump_pddl(self):
    return "(or"

class QuantifiedCondition(Condition):
  def __init__(self, parameters, parts):
    self.parameters = tuple(parameters)
    self.parts = tuple(parts)
    self.hash = hash((self.__class__, self.parameters, self.parts))
  def __eq__(self, other):
    # Compare hash first for speed reasons.
    return (self.hash == other.hash and
            self.__class__ is other.__class__ and
            self.parameters == other.parameters and
            self.parts == other.parts)
  def copy(self):
    return self.__class__(list(self.parameters), [p.copy() for p in self.parts])
  def _dump(self, indent="  "):
    arglist = ", ".join(map(str, self.parameters))
    return "%s %s" % (self.__class__.__name__, arglist)
  def _dump_pddl(self):
    arglist = " ".join(map(types.pddl_str, self.parameters)).replace(':', " -")
    # this is used and overridden in the subclasses
    return arglist  
  def _simplified(self, parts):
    if isinstance(parts[0], ConstantCondition):
      return parts[0]
    else:
      return self._propagate(parts)
  def uniquify_variables(self, type_map, renamings={}):
    renamings = dict(renamings) # Create a copy.
    new_parameters = [par.uniquify_name(type_map, renamings)
                      for par in self.parameters]
    new_parts = (self.parts[0].uniquify_variables(type_map, renamings),)
    return self.__class__(new_parameters, new_parts)

  def free_variables(self):
    result = Condition.free_variables(self)
    for par in self.parameters:
      result.discard(par.name)
    return result
  def change_parts(self, parts):
    return self.__class__(self.parameters, parts)

class UniversalCondition(QuantifiedCondition):
  def _untyped(self, parts):
    type_literals = [NegatedAtom(par.type, [par.name]) for par in self.parameters]
    return UniversalCondition(self.parameters,
                              [Disjunction(type_literals + parts)])
  def negate(self):
    return ExistentialCondition(self.parameters, [p.negate() for p in self.parts])
  def has_universal_part(self):
    return True
  def _dump_pddl(self):
    return "(forall (%s)" % QuantifiedCondition._dump_pddl(self)

class ExistentialCondition(QuantifiedCondition):
  def _untyped(self, parts):
    type_literals = [Atom(par.type, [par.name]) for par in self.parameters]
    return ExistentialCondition(self.parameters,
                                [Conjunction(type_literals + parts)])
  def negate(self):
    return UniversalCondition(self.parameters, [p.negate() for p in self.parts])
  def instantiate(self, var_mapping, init_facts, fluent_facts, result):
    assert not result, "Condition not simplified"
    self.parts[0].instantiate(var_mapping, init_facts, fluent_facts, result)
  def has_existential_part(self):
    return True
  def _dump_pddl(self):
    return "(exists (%s)" % QuantifiedCondition._dump_pddl(self)

def treat_initially(predicate, args, value):
  predicate, args = "i__"+args[0][0], args[0][1:]
  return predicate, args, value

def treat_knowledge(predicate, args, value):
  assert predicate == K_PREFIX
  real_predicate = args[-1]
  assert isinstance(real_predicate, list), "%s is not a list, but a %s. val=%s" % (real_predicate, type(real_predicate), value)
  args = args[:-1] + real_predicate[1:]
  predicate = "%s__%s" % (predicate, real_predicate[0])
  return predicate, args, value

def treat_svar_domain(predicate, args, value):
  assert predicate == DOMAIN_KW
  assert len(args)==3 and args[1] == ":", "svar domain definition does not use correct syntax '(domain (svar arg1 .. argk) : (val1 .. valn))'"
  from predicates import Predicate
  predicate = Predicate.svar_domain_predicate_name(args[0][0])
  value = args[2]
  args = args[0][1:]
  return predicate, args, value

def treat_svar_in_domain(predicate, args, value):
  assert predicate == IN_DOMAIN_KW
  assert len(args)==2, "in_domain atom does not use correct syntax '(in_domain (svar arg1 .. argk) val)'"
  from predicates import Predicate
  predicate = Predicate.svar_in_domain_predicate_name(args[0][0])
  args = args[0][1:] + [args[1]]
  value = None
  return predicate, args, value

def treat_negation(predicate, args, value):
  if value is None or value == TRUE_TUP:
    value = FALSE_TUP
  elif value == FALSE_TUP:
    value = TRUE_TUP
  predicate = args[0][0]
  args = args[0][1:]
  return predicate, args, value
  
# SPECIAL_PREDICATES = {K_PREFIX : treat_knowledge, "initially" : treat_initially, DOMAIN_KW : treat_svar_domain, IN_DOMAIN_KW : treat_svar_in_domain, "not" : treat_negation}

def head_tail(alist):
  return alist[0], alist[1:]

def parse_literal(alist, negated=False):
  """ cases:
  1. proposition: (bigger o1 o2)
  2. negated proposition: (not (bigger o1 o2))
  3. state var assignment: (pos x : y)
  4. belief/mutual belief: (K (agt1 agt2) (pos x : y))
  5. KVAL fact: (KVAL (agt1 agt2) (pos x))
  6. in-domain fact
  """
  modality = ""
  believers = set()
  value = None
  head, args = head_tail(alist)
  if head == "not":
    # TODO: this is the old, faulty implementation. remove when removing the tuple representation for values
    print "warning: using 'not' in MAPL files is currently not safe." 
    assert len(alist) == 2
    return parse_literal(ars, not negated)
  if head in ("k", "kval"):
    assert len(args) == 2
    modality = head
    assert not (modality == "kval" and ":" in args), "alist %s seems buggy" % alist
    believers = args[0]
    assert isinstance(believers, list)
    head, args = head_tail(args[1])
  if ":" in args:
    assert args.index(":") == len(args) - 2
    value = args[-1]
    args = args[:-2]
  else:
    value = FALSE_STRING if negated else TRUE_STRING   
  return Atom(head, args, value, believers, modality)      # TODO: negation?



class Literal(Condition):
  parts = []
  def __init__(self, predicate, args, value, believers, modality=""):
    Condition.__init__(self, ())
    self.predicate = predicate
    if isinstance(value, basestring):
      value = (value,)
    self.value = tuple(value)
    self.args = tuple(args)
    self.believers = tuple(sorted(believers))
    self.modality = modality
    try:
      self.hash = hash((self.__class__, self.believers, self.predicate, self.args, self.value, self.modality))
    except:
      print str(self)
      raise
  def copy(self):
    return self.__class__(self.predicate, self.args, self.value, self.believers, self.modality)
  def __str__(self):
    return "Literal: %s %s, %s, %s, %s" % (self.modality, self.believers, self.predicate, self.args, self.value)
  def _dump(self):
    return str(self)
  
  def _dump_pddl(self, use_direct_k=False, drop_boolean_values=False):
    """format: (modality__predicate believers value)"""
    pred = self.predicate
    val = self.value[0]  # TODO: remove this when dropping tuple representation
    if pred == "=":
      drop_boolean_values = True
    if drop_boolean_values:
      val = ""
    if self.modality:
      mod = self.modality
      if use_direct_k and mod == "kval":
        mod = "kvald"
      pred = "%s__%s" % (mod, pred)
    args = itertools.chain([pred], self.believers, self.args, [val])
    strs = [str(arg) for arg in args if arg]    
    s = "(%s)" % (" ".join(strs))
    if drop_boolean_values and val == FALSE_STRING:
      s = "(not %s)" % s
    return s
    

  def _dump_mapl(self):
    s = self._dump_pddl(keep_colon=True)
    return s

  def is_negated(self):
    return self.value == FALSE_TUP or self.__class__ == NegatedAtom
  
  def dump_pddl_initially(self, stream, indent):
    print >>stream, self.pddl_str_initially(indent)

  def pddl_str_initially(self, indent):
    return self.pddl_str(indent).replace(self.predicate, "i__"+self.predicate, 1)

  def mapsim_svar(self, replacements={}):
    args = [replacements.get(v,v) for v in self.args]
    svar = state.StateVariable(self.predicate, args, "some_return_type")
    instantiated_svar = state.InstantiatedStateVariable(svar, args, self.believers, self.modality)
    return instantiated_svar

  def as_svar_assignment(self, replacements={}):
    svar = self.mapsim_svar(replacements)
    val = self.value
    if self.__class__ == NegatedAtom:
      print "NegatedAtom should not be used..."
      sys.exit()
    assert isinstance(val, tuple)  # TODO: do we really still need tuples???
    val = tuple(replacements.get(v,v) for v in val)
    return svar, val

  def as_svar_assignments(self, replacements={}):
    return [self.as_svar_assignment(replacements)]

  def mapl_str(self, add_parens=False):
    svar, val = self.as_svar_assignment()
    return state.str_assignment(svar, val, add_parens=add_parens)

  def change_parts(self, parts):
    return self
  def uniquify_variables(self, type_map, renamings={}):
    return self.rename_variables(renamings)
  def rename_variables(self, renamings):
    new_args = [renamings.get(arg, arg) for arg in self.args]
    new_val = [renamings.get(arg, arg) for arg in self.value]
    new_believers = [renamings.get(arg, arg) for arg in self.believers]
    return self.__class__(self.predicate, new_args, new_val, new_believers, self.modality)
  def free_variables(self):
    return sets.Set([arg for arg in self.args if arg[0] == "?"])


class Atom(Literal):
  pass
##   negated = False
##   def to_untyped_strips(self):
##     return [self]
##   def instantiate(self, var_mapping, init_facts, fluent_facts, result):
##     args = [var_mapping.get(arg, arg) for arg in self.args]
##     atom = Atom(self.predicate, args)
##     if atom in fluent_facts:
##       result.append(atom)
##     elif atom not in init_facts:
##       raise Impossible()
##   def negate(self):
##     return NegatedAtom(self.predicate, self.args)
##   def positive(self):
##     return self

class NegatedAtom(Literal):
  pass
##   negated = True
##   def __init__(self, predicate, args, value=FALSE_TUP):
##     Literal.__init__(self, predicate, args, value)
##   def _relaxed(self, parts):
##     return Truth()
##   def instantiate(self, var_mapping, init_facts, fluent_facts, result):
##     args = [var_mapping.get(arg, arg) for arg in self.args]
##     atom = Atom(self.predicate, args)
##     if atom in fluent_facts:
##       result.append(NegatedAtom(self.predicate, args))
##     elif atom in init_facts:
##       raise Impossible()
##   def negate(self):
##     return Atom(self.predicate, self.args)
##   positive = negate
