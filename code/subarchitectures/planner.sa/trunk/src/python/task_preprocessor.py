from collections import defaultdict
import itertools
import re
from string import maketrans

import standalone
from standalone.task import Task  # requires standalone planner to be in PYTHONPATH already
from standalone import pddl
from standalone.pddl import state, prob_state

from de.dfki.lt.tr.beliefs.slice.sitbeliefs import dBelief
from de.dfki.lt.tr.beliefs.slice import logicalcontent, distribs

log = standalone.config.logger("PythonServer")

forbidden_letters = "-: "
replace_chr = "_"
trans_tbl = maketrans(forbidden_letters, replace_chr * len(forbidden_letters))
UCASE_REXP = re.compile("([A-Z])")

# attention: current_domain is used as a global in this module
current_domain = None
belief_dict = None

class SVarDistribution(tuple):
  def __new__(_class, feat, args, value):
    return tuple.__new__(_class, [feat] + args + [value])
    
  feature = property(lambda self: self[0])
  args = property(lambda self: self[1:-1])
  values = property(lambda self: self[-1])

  def __str__(self):
    valstr = " ".join("%.2f: %s" % (v[1], v[0]) for v in self.values)
    return "(%s %s) = [%s]" % (self.feature, " ".join(a.name for a in self.args), valstr)

def rename_objects(objects):
  namedict = {}
  for obj in objects:
    oldname = obj.name
    new_name = "%s_%s" % (obj.type, obj.name.translate(trans_tbl))
    #the cast ids are case sensitive, so we have to replace uppercase chars
    #with something different
    new_name = re.sub(UCASE_REXP, r'_\1', new_name).lower()
    #obj.name = obj.name.lower()
    obj.rename(new_name)
  
    namedict[oldname] = obj
    namedict[obj.name] = oldname
    
  return namedict

def transform_goal_string(goal, namedict):
  for name, obj in namedict.iteritems():
    if not isinstance(obj, pddl.TypedObject):
      continue
    goal=goal.replace("'%s'" % name, obj.name)

  return goal

def belief_to_object(belief):
  object_type = pddl.t_object
  #get type hints from belief.type
  if belief.type.lower() in current_domain.types:
    object_type = current_domain.types[belief.type.lower()]

  return pddl.TypedObject(belief.id, object_type)
    
def feature_val_to_object(fval):
  if fval.__class__ == logicalcontent.ElementaryFormula:
    val = fval.prop.lower()
    #lookup constants
    #if val in current_domain:
    #  return current_domain[val]
    if "unknown" in val:
      return pddl.UNKNOWN

    return pddl.TypedObject(val, pddl.t_object)
  
  elif fval.__class__ == logicalcontent.PointerFormula:
    bel = belief_dict[fval.pointer.id]
    return belief_to_object(bel)
  
  elif fval.__class__ == logicalcontent.IntegerFormula:
    if "numeric-fluents" in current_domain.requirements or "fluents" in current_domain.requirements:
      return pddl.TypedObject(fval.val, pddl.t_number)
    else:
      return pddl.TypedObject(str(fval.val), pddl.t_object)
  
  elif fval.__class__ == logicalcontent.BooleanFormula:
    if fval.val:
      return pddl.TRUE
    return pddl.FALSE
  
  elif fval.__class__ == logicalcontent.UnknownFormula:
    return pddl.UNKNOWN

  return None
  #assert False, "Unknown feature type: %s" % fval.__class__


def gen_fact_tuples(beliefs):
  x = logicalcontent.PointerFormula(None)
  def extract_features(dist):
    # if isinstance(dist, distribs.DistributionWithExistDep):
    #   #ignore existence probability for now
    #   return extract_features(dist.Pc)
    if isinstance(dist, distribs.CondIndependentDistribs):
      result = []
      for feat, fval_dist in dist.distribs.iteritems():
        assert isinstance(fval_dist, distribs.BasicProbDistribution)
        assert feat == fval_dist.key
        value = fval_dist.values
        
        if isinstance(value, distribs.FormulaValues):
          for valpair in value.values:
            val = feature_val_to_object(valpair.val)
            if val is not None:
              log.debug("%s = %s:%.2f", feat, val, valpair.prob)
              result.append((feat, val, valpair.prob))
        elif isinstance(value, distribs.NormalValues):
          #TODO: discretize?
          val = feature_val_to_object(value.mean)
          if val is not None:
            log.debug("%s = %s", feat, val)
            result.append((feat, val , 1.0))
      return result
    assert False, "class %s of %s not supported" % (str(type(dist)), str(dist))

  for bel in beliefs:
    factdict = defaultdict(list)
    for feat, val, prob in extract_features(bel.content):
      factdict[str(feat)].append((val, prob))
      
    if bel.type != "relation":
      obj = belief_to_object(bel)
      for feat,vals in factdict.iteritems():
        log.debug("(%s %s) = %s : %f", feat, obj, val, prob)
        yield SVarDistribution(feat, [obj], vals)
    else:
      elems = []
      i=0
      while ("val%d" % i) in factdict:
        el_vals = factdict["val%d" % i]
        assert len(el_vals) == 1, "valN features in relations must have exactly one possible value"
        elems.append(el_vals[0][0])
        i += 1

      for feat,vals in factdict.iteritems():
        if feat.startswith("val"):
          continue
        log.debug("(%s %s) = %s : %f", feat, " ".join(map(str, elems)), vals[0][0], vals[0][1])
        yield SVarDistribution(feat, elems, vals)

def filter_unknown_preds(fact_tuples):
  for ft in fact_tuples:
    if ft.feature not in current_domain.functions and \
          ft.feature not in current_domain.predicates:
      log.debug("filtering feature assignment %s, because '%s' is not part of the planning domain", \
                    str(ft), str(ft.feature))
    else:
      #print "using", map(str, ft)
      yield ft

def tuples2facts(fact_tuples):
  for ftup in fact_tuples:
    feature_label = str(ftup.feature)
    if feature_label in current_domain.functions:
      func = current_domain.functions.get(feature_label, ftup.args)
    else:
      assert feature_label in current_domain.predicates
      func = current_domain.predicates.get(feature_label, ftup.args)

    if not isinstance(func, pddl.Function):
      log.warning("Error looking up %s(%s), got %s", feature_label, ", ".join(map(str, ftup.args)), str(func))
      continue
      
    if len(ftup.values) == 1:
      yield state.Fact(state.StateVariable(func, ftup.args), ftup.values[0][0])
    else:
      vdist = prob_state.ValueDistribution(dict(ftup.values))
      yield prob_state.ProbFact(state.StateVariable(func, ftup.args), vdist)

def unify_objects(obj_descriptions):
  namedict = {}
  for ftup in obj_descriptions:
    args = []
    for obj in ftup.args:
      if obj.name in namedict:
        args.append(namedict[obj.name])
      else:
        namedict[obj.name] = obj
        args.append(obj)
    values = []
    for obj, prob in ftup.values:
      if obj.name in namedict:
        values.append((namedict[obj.name], prob))
      else:
        namedict[obj.name] = obj
        values.append((obj, prob))

    yield SVarDistribution(ftup.feature, args, values)
      
def infer_types(obj_descriptions):
  constraints = defaultdict(set)
  for ftup in obj_descriptions:
    pred = str(ftup.feature)
    if pred in current_domain.functions:
      declarations = current_domain.functions[pred]
      is_function = True
    else:
      declarations = current_domain.predicates[pred]
      is_function = False
    for declaration in declarations:
      ftypes = map(lambda a: a.type, declaration.args)
      #ftypes.append(declaration.type)

      #only consider this function if the basic value types (object, boolean, number) match
      if len(ftup.args) == len(ftypes) and \
            all(t.equal_or_subtype_of(a.type) or a.type.equal_or_subtype_of(t) for (t,a) in zip(ftypes, ftup.args)) and \
            all(declaration.type.equal_or_subtype_of(arg.type) for arg, _ in ftup.values):
        for arg, type in zip(ftup.args, ftypes):
          constraints[arg].add(type)
        for arg, _ in ftup.values:
          constraints[arg].add(declaration.type)
          
        #print "Inferring: %s is instance of %s because of use in %s" % (name, nametype, declaration)
        #print "Inferring: %s is instance of %s because of use in %s" % (val, valtype, declaration)
        
  objects = set()
  for obj in constraints:
    if obj == pddl.UNKNOWN:
      continue
    
    # now find most specific type
    
    # how could THAT happen?
    #if obj not in constraints:
    #  constraints[obj].add("object")
    
    types = constraints[obj]
    types.add(obj.type)
    #log.debug("%s has the following type constraints %s", obj.name, map(str,types))
    def type_cmp(type1, type2):
      if type1 == type2:
        return 0
      elif type1.is_subtype_of(type2):
        return -1
      elif type2.is_subtype_of(type1):
        return 1
      log.error("%s could be of types %s or %s", obj.name, type1, type2)
      assert False, "Multiple inheritance not supported yet"
    most_spec_type = sorted(types, cmp=type_cmp)[0]
    obj.change_type(most_spec_type)

    if obj in current_domain and current_domain[obj].is_instance_of(most_spec_type):
      #use existing constant
      obj.change_type(current_domain[obj].type)
      continue

    objects.add(obj)

  return objects
