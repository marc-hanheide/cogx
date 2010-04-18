from collections import defaultdict
import itertools
import re
from string import maketrans

import standalone
from standalone.task import Task  # requires standalone planner to be in PYTHONPATH already
from standalone import pddl
from standalone.pddl import state, prob_state
from beliefmodels.autogen import distribs, featurecontent

log = standalone.config.logger("PythonServer")

forbidden_letters = "-:"
replace_chr = "_"
trans_tbl = maketrans(forbidden_letters, replace_chr * len(forbidden_letters))
UCASE_REXP = re.compile("([A-Z])")

# attention: current_domain is used as a global in this module
current_domain = None

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
    obj.name = "%s_%s" % (obj.type, obj.name.translate(trans_tbl))
    #the cast ids are case sensitive, so we have to replace uppercase chars
    #with something different
    obj.name = re.sub(UCASE_REXP, r'_\1', obj.name).lower()
    #obj.name = obj.name.lower()
  
    namedict[oldname] = obj
    namedict[obj] = oldname
    
  return namedict

def transform_goal_string(goal, namedict):
  for name, obj in namedict.iteritems():
    if not isinstance(name, str):
      continue
    goal=goal.replace("'%s'" % name, obj.name)

  return goal

def feature_val_to_object(fval):
  if fval.__class__ == featurecontent.StringValue:
    val = fval.val.lower()
    #lookup constants
    if val in current_domain:
      return current_domain[val]
    if val == "unknown":
      return pddl.UNKNOWN

    return pddl.TypedObject(val, pddl.t_object)
  
  elif fval.__class__ == featurecontent.PointerValue:
    #todo: how to support address values sensibly?
    return pddl.TypedObject(fval.beliefId, pddl.t_object)
  
  elif fval.__class__ == featurecontent.IntegerValue:
    return pddl.TypedObject(fval.val, pddl.t_number)
  
  elif fval.__class__ == featurecontent.BooleanValue:
    if fval.val:
      return pddl.TRUE
    return pddl.FALSE
  
  elif fval.__class__ == featurecontent.UnknownValue:
    return pddl.UNKNOWN

  assert False, "Unknown feature type: %s" % fval.__class__


def gen_fact_tuples(beliefs):
  def extract_features(dist):
    if isinstance(dist, distribs.DistributionWithExistDep):
      #ignore existence probability for now
      return extract_features(dist.Pc)
    if isinstance(dist, distribs.CondIndependentDistribs):
      result = []
      for feat, fval_dist in dist.distribs.iteritems():
        assert isinstance(fval_dist, distribs.BasicProbDistribution)
        assert feat == fval_dist.key
        value = fval_dist.values
        
        if isinstance(value, distribs.FeatureValues):
          for valpair in value.values:
            val = feature_val_to_object(valpair.val)
            result.append((feat, val, valpair.prob))
        elif isinstance(value, distribs.NormalValues):
          #TODO: discretize?
          result.append((feat, feature_val_to_object(value.mean), 1.0))
      return result
    if isinstance(dist, distribs.DiscreteDistribution):
      assert False, "DiscreteDistribution not supported yet"
    assert False, "class %s of %s not supported" % (str(type(dist)), str(dist))

  for bel in beliefs:
    factdict = defaultdict(list)
    for feat, val, prob in extract_features(bel.content):
      print feat, val, prob
      factdict[str(feat)].append((val, prob))
      
    if bel.type != "relation":
      obj = pddl.TypedObject(bel.id, pddl.t_object)
      for feat,vals in factdict.iteritems():
        yield SVarDistribution(feat, [obj], vals)
    else:
      elems = []
      i=0
      while ("element%d" % i) in factdict:
        el_vals = factdict["element%d" % i]
        assert len(el_vals) == 1, "element features in relations must have exactly one possible value"
        elems.append(el_vals[0][0])
        i += 1

      for feat,vals in factdict.iteritems():
        if feat.startswith("element"):
          continue
        yield SVarDistribution(feat, elems, vals)
  # for bel in beliefs:
  #   if isinstance(union, specialentities.RelationUnion):
  #     try:
  #       source = feature_val_to_object(union.usource.alternativeValues[0])
  #       target = feature_val_to_object(union.utarget.alternativeValues[0])
  #     except Exception, e:
  #       print "Error getting source or target of relation %s." % union.entityID
  #       print "Message was: %s" % str(e)
  #       continue

  #     for feature in union.features:
  #       # choose feature val with highest probability:
  #       max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb)
  #       yield (feature.featlabel, source, target, feature_val_to_object(max_val))
        
  #   else:
  #     name = union.entityID
  #     object = pddl.TypedObject(name, pddl.t_object)
      
  #     for feature in union.features:
  #       # choose feature val with highest probability:
  #       max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb)
  #       yield (feature.featlabel, object, feature_val_to_object(max_val))

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
      if len(ftup.args) == len(ftypes) and all(map(lambda t, a: t.equal_or_subtype_of(a.type), ftypes, ftup.args)) and \
            all(declaration.type.equal_or_subtype_of(arg.type) for arg, _ in ftup.values):
        for arg, type in zip(ftup.args, ftypes):
          constraints[arg].add(type)
        for arg, _ in ftup.values:
          constraints[arg].add(declaration.type)
          
        #print "Inferring: %s is instance of %s because of use in %s" % (name, nametype, declaration)
        #print "Inferring: %s is instance of %s because of use in %s" % (val, valtype, declaration)
        
  objects = set()
  for obj in constraints:
    if obj == pddl.UNKNOWN or obj in current_domain:
      #don't change any constants
      continue
      
    # now find most specific type
    
    # how could THAT happen?
    #if obj not in constraints:
    #  constraints[obj].add("object")
    
    types = constraints[obj]
    #print "%s has the following type constraints %s" % (obj.name, map(str,types))
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
    obj.type = most_spec_type
    objects.add(obj)

  return objects

def generate_mapl_task(task_desc, domain_fn):
  global current_domain
  task = Task(task_desc.id)
  
  task.load_mapl_domain(domain_fn)
  current_domain = task._mapldomain
  
  obj_descriptions = list(unify_objects(filter_unknown_preds(gen_fact_tuples(task_desc.state))))
  
  objects = infer_types(obj_descriptions)
  task.namedict = rename_objects(objects)

  facts = list(tuples2facts(obj_descriptions))

  problem = pddl.Problem("cogxtask", objects, [], None, task._mapldomain)
  try:
    goalstrings = transform_goal_string(task_desc.goal, task.namedict).split("\n")
    problem.goal = pddl.parser.Parser.parse_as(goalstrings, pddl.conditions.Condition, problem)
    log.debug("goal: %s", problem.goal)
  except pddl.parser.ParseError:
    problem.goal = pddl.conditions.Falsity()

  task._mapltask = problem
  task.set_state(prob_state.ProbabilisticState(facts, problem).determinized_state(0.1, 0.9))
  
  return task  

def generate_mapl_state(task_desc, task):
  global current_domain
  current_domain = task._mapldomain
  
  obj_descriptions = list(unify_objects(filter_unknown_preds(gen_fact_tuples(task_desc.state))))
  
  objects = infer_types(obj_descriptions)
  task.namedict = rename_objects(objects)

  facts = list(tuples2facts(obj_descriptions))
  state = prob_state.ProbabilisticState(facts, task.mapltask).determinized_state(0.1, 0.9) 
  return objects, state


def map2binder_rep(plan, task):
  assert task.namedict, "task has no namedict"
  nd = task.namedict
  print "nd:", nd
  new_plan = plan.copy()
  for anode in new_plan.all_actions():
    for a in anode.args:
      print a.name, ":", nd.get(a, a.name)
    print "old action", anode
    anode.args = [nd.get(a, a.name) for a in anode.args]
    print "new action", anode
  return new_plan
  
