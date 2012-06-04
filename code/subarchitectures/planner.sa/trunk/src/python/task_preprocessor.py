from collections import defaultdict
import re
from string import maketrans

import standalone
from standalone import pddl
from standalone.pddl import state, prob_state

from de.dfki.lt.tr.beliefs.slice import logicalcontent, distribs
import eu.cogx.beliefs.slice as cogxbm

log = standalone.config.logger("belief-preprocessor")

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
    return "(%s %s) = [%s]" % (self.feature, " ".join(a.name for a in
    self.args), valstr)

class AttributedSVarDistribution(SVarDistribution):
  def __new__(_class, agent, feat, args, value):
    return tuple.__new__(_class, [agent, feat] + args + [value])

  agent = property(lambda self: self[0])
  feature = property(lambda self: self[1])
  args = property(lambda self: self[2:-1])
  values = property(lambda self: self[-1])

  def __str__(self):
    valstr = " ".join("%.2f: %s" % (v[1], v[0]) for v in self.values)
    return "(attributed %s (%s %s)) = [%s]" % (self.agent, self.feature, " ".join(a.name for a in self.args), valstr)

def rename_objects(objects, existing=set(), add_renamings={}):
  castname_to_obj = {}
  obj_to_castname = {}
  for obj in objects:
    if obj in existing:
      continue
    oldname = obj.name
    if oldname in add_renamings:
      new_name = add_renamings[oldname]
    else:
      new_name = "%s_%s" % (obj.type, obj.name.translate(trans_tbl))
      #the cast ids are case sensitive, so we have to replace uppercase chars
      #with something different
      new_name = re.sub(UCASE_REXP, r'_\1', new_name).lower()
    obj.rename(new_name)
  
    castname_to_obj[oldname] = obj
    obj_to_castname[obj] = oldname
    
  return castname_to_obj, obj_to_castname

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
    if fval.pointer.id not in belief_dict:
      log.warning("Pointer to belief %s cannot be resolved!", fval.pointer.id)
      return pddl.UNKNOWN
    bel = belief_dict[fval.pointer.id]
    return belief_to_object(bel)
  
  elif fval.__class__ in (logicalcontent.IntegerFormula, logicalcontent.FloatFormula):
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
  def extract_features(dist):
    # if isinstance(dist, distribs.DistributionWithExistDep):
    #   #ignore existence probability for now
    #   return extract_features(dist.Pc)
    if isinstance(dist, distribs.CondIndependentDistribs):
      for feat, fval_dist in dist.distribs.iteritems():
        if feat == "given":
          continue
        assert isinstance(fval_dist, distribs.BasicProbDistribution)
        assert feat == fval_dist.key
        value = fval_dist.values
        
        if isinstance(value, distribs.FormulaValues):
          if len(value.values) == 0:
              yield (feat, pddl.UNKNOWN, 1.0)
          for valpair in value.values:
            if isinstance(valpair.val, logicalcontent.NegatedFormula):
              val = feature_val_to_object(valpair.val.negForm)
              prob = 0.0
            else:
              val = feature_val_to_object(valpair.val)
              prob = valpair.prob
            if val is not None:
              #log.debug("%s = %s:%.2f", feat, val, prob)
              yield (feat, val, prob)
        elif isinstance(value, distribs.NormalValues):
          #TODO: discretize?
          val = feature_val_to_object(value.mean)
          if val is not None:
            #log.debug("%s = %s", feat, val)
            yield (feat, val , 1.0)


  def decode_relation(dist):
    factdict = defaultdict(list)
    for feat, val, prob in extract_features(dist):
      factdict[str(feat)].append((val, prob))

    elems = []
    i=0
    while ("val%d" % i) in factdict:
      el_vals = factdict["val%d" % i]
      assert len(el_vals) == 1, "valN features in relations must have exactly one possible value"
      elems.append(el_vals[0][0])
      i += 1

    if any(val == pddl.UNKNOWN for val in elems):
      return

    for feat,vals in factdict.iteritems():
      if feat.startswith("val"):
        continue
      #log.debug("(%s %s) = %s : %f", feat, " ".join(map(str, elems)), vals[0][0], vals[0][1])
      yield SVarDistribution(feat, elems, vals)
      
  for bel in beliefs:
    # print "Belief:", bel.id
    # print "   ep. status:", type(bel.estatus)
    
    #always treat CondIndependentDistribList as list of relations
    if isinstance(bel.content, distribs.CondIndependentDistribList):
      # print "   is list"
      for dist in bel.content.distribs:
        for fact in decode_relation(dist):
          yield fact

    elif bel.type == "relation":
      if isinstance(bel, cogxbm.HypotheticalBelief):
        #TODO: which hypothetical relations so we want to keep?
        continue
      # print "   is relation"
      for fact in decode_relation(bel.content):
        yield fact

    else:
      factdict = defaultdict(list)
      attributed_object = None
      for feat, val, prob in extract_features(bel.content):
        # print feat, val, prob
        if feat == "about":
          attributed_object = val
        factdict[str(feat)].append((val, prob))

      if attributed_object is not None and isinstance(bel, cogxbm.HypotheticalBelief):
        #Don't put hypotheses about existing objects into the state
        continue
      elif attributed_object is not None:
        yield SVarDistribution("existence",  [attributed_object], [])
        agent = pddl.TypedObject("tutor",pddl.mapl.t_agent)
        for feat,vals in factdict.iteritems():
          # print "attributed:", agent, feat, attributed_object, vals
          yield AttributedSVarDistribution(agent, feat, [attributed_object], vals)
      else:
        obj = belief_to_object(bel)
        if isinstance(bel, cogxbm.HypotheticalBelief):
          yield SVarDistribution("is-virtual", [obj], [(pddl.TRUE, 1.0)])
        else:
          yield SVarDistribution("entity-exists",  [obj], [(pddl.TRUE, 1.0)])

        for feat,vals in factdict.iteritems():
          # for val, prob in vals:
          #   log.debug("(%s %s) = %s : %f", feat, obj, val, prob)
          yield SVarDistribution(feat, [obj], vals)
    # else:
    #   elems = []
    #   i=0
    #   while ("val%d" % i) in factdict:
    #     el_vals = factdict["val%d" % i]
    #     assert len(el_vals) == 1, "valN features in relations must have exactly one possible value"
    #     elems.append(el_vals[0][0])
    #     i += 1

    #   if any(val == pddl.UNKNOWN for val in elems):
    #     continue

    #   # print map(str, elems)
        
    #   for feat,vals in factdict.iteritems():
    #     if feat.startswith("val"):
    #       continue
    #     #log.debug("(%s %s) = %s : %f", feat, " ".join(map(str, elems)), vals[0][0], vals[0][1])
    #     yield SVarDistribution(feat, elems, vals)

def filter_unknown_preds(fact_tuples):
  for ft in fact_tuples:
    if ft.feature != "existence" and\
          ft.feature not in current_domain.functions and \
          ft.feature not in current_domain.predicates:
      pass
      log.trace("filtering feature assignment %s, because '%s' is not part of the planning domain", \
                   str(ft), str(ft.feature))
    else:
      # print "using", map(str, ft)
      yield ft

def tuples2facts(fact_tuples):
  for ftup in fact_tuples:
    feature_label = str(ftup.feature)
    if feature_label == "existence":
      continue
    elif feature_label in current_domain.functions:
      func = current_domain.functions.get(feature_label, ftup.args)
    else:
      assert feature_label in current_domain.predicates
      func = current_domain.predicates.get(feature_label, ftup.args)

    if not isinstance(func, pddl.Function):
      log.warning("Error looking up %s(%s), got %s", feature_label, ", ".join(map(str, ftup.args)), str(func))
      continue

    if len(ftup.values) == 1 and ftup.values[0][1] == 1.0:
      if isinstance(ftup, AttributedSVarDistribution):
        yield state.Fact(state.StateVariable(func, ftup.args,\
                                              modality=pddl.mapl.attributed, modal_args = [ftup.agent,ftup.values[0][0]]),pddl.TRUE)
      else:
        yield state.Fact(state.StateVariable(func, ftup.args),ftup.values[0][0])
    elif len(ftup.values) == 1 and ftup.values[0][1] == 0.0 and isinstance(ftup, AttributedSVarDistribution):
      yield state.Fact(state.StateVariable(func, ftup.args,\
                                              modality=pddl.mapl.neg_attributed, modal_args = [ftup.agent,ftup.values[0][0]]),pddl.TRUE)
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

    if isinstance(ftup, AttributedSVarDistribution):
      if ftup.agent in namedict:
        agent = namedict[ftup.agent.name]
      else:
        namedict[ftup.agent.name] = ftup.agent
        agent = ftup.agent
      yield AttributedSVarDistribution(agent, ftup.feature, args, values)
    else:
      yield SVarDistribution(ftup.feature, args, values)
    
      
def infer_types(obj_descriptions):
  constraints = defaultdict(set)
  for ftup in obj_descriptions:
    pred = str(ftup.feature)
    if pred == "existence":
      arg = ftup.args[0]
      if arg.type != pddl.t_object:
        constraints[arg].add(frozenset([arg.type]))
      continue
    elif pred in current_domain.functions:
      declarations = current_domain.functions[pred]
      #is_function = True
    else:
      declarations = current_domain.predicates[pred]
      #is_function = False

    possible_types = defaultdict(set)
    for declaration in declarations:
      ftypes = map(lambda a: a.type, declaration.args)
      #ftypes.append(declaration.type)

      #only consider this function if the basic value types (object, boolean, number) match
      if len(ftup.args) == len(ftypes) and \
            all(t.equal_or_subtype_of(a.type) or a.type.equal_or_subtype_of(t) for (t,a) in zip(ftypes, ftup.args)) and \
            all(declaration.type.equal_or_subtype_of(arg.type) for arg, _ in ftup.values):
        for arg, type in zip(ftup.args, ftypes):
          possible_types[arg].add(type)
          #print "Inferring: %s is instance of %s because of use in %s" % (arg.name, str(type), str(declaration))
        for arg, _ in ftup.values:
          possible_types[arg].add(declaration.type)
          #print "Inferring: %s is instance of %s because of use in %s" % (arg.name, str(declaration.type), str(declaration))
          
    for arg, types in possible_types.iteritems():
      #print "%s induces the following possible types for %s: %s" % (declaration.name,  arg.name, ", ".join(t.name for t in types))
      #types.add(arg.type) # always add the basetype as an "ignore-this-feature" option
      constraints[arg].add(frozenset(types))
        
  objects = set()
  for obj in constraints:
    if obj == pddl.UNKNOWN:
      continue
    
    # now find most specific type
    
    # how could THAT happen?
    #if obj not in constraints:
    #  constraints[obj].add("object")
    
    typesets = list(set(ts) for ts in constraints[obj])
    typesets.append(set([obj.type]))
    types = set()
    for ts in typesets:
      for ts2 in typesets:
        subset = set(t for t in ts if any(t.is_compatible(t2) for t2 in ts2))
        subset |= set(t for t in ts2 if any(t.is_compatible(t2) for t2 in ts))
        assert subset, "incompatible types: (%s) / (%s)" % (", ".join(str(t) for t in ts), ", ".join(str(t) for t in ts2))
        ts &= subset
        ts2 &= subset
          
    types = reduce(lambda x,y: x | y, typesets, set())
        
    # for ts in typesets:
    #   candidates = set()
    #   for t in ts:
    #     #eleminate all impossible types from this typeset
    #     if any(t.is_compatible(t2) for t2 in types):
    #       candidates.add(t)
    #   for t in set(types):
    #     if not any(t.is_compatible(t2) for t2 in candidates):
    #       types.discard(t)
    #   types |= candidates
      
    #types.add(frozenset([obj.type]))
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
