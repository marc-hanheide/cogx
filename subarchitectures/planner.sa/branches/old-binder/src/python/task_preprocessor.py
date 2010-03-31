from collections import defaultdict
import itertools
import re
from string import maketrans

from standalone.task import Task  # requires standalone planner to be in PYTHONPATH already
from standalone import pddl
from standalone.pddl import state
import binder.autogen
import binder.autogen.specialentities as specialentities
import binder.autogen.featvalues as featvalues

forbidden_letters = "-:"
replace_chr = "_"
trans_tbl = maketrans(forbidden_letters, replace_chr * len(forbidden_letters))
UCASE_REXP = re.compile("([A-Z])")

# attention: current_domain is used as a global in this module
current_domain = None

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
  if fval.__class__ == featvalues.StringValue:
    #lookup constants
    if fval.val in current_domain:
      return current_domain[fval.val]
    if fval.val == "unknown":
      return pddl.UNKNOWN
    
    return pddl.TypedObject(fval.val, pddl.t_object)
  
  elif fval.__class__ == featvalues.AddressValue:
    #todo: how to support address values sensibly?
    return pddl.TypedObject(fval.val, pddl.t_object)
  
  elif fval.__class__ == featvalues.IntegerValue:
    return pddl.TypedObject(fval.val, pddl.t_number)
  
  elif fval.__class__ == featvalues.BooleanValue:
    if fval.val:
      return pddl.TRUE
    return pddl.FALSE
  
  elif fval.__class__ == featvalues.UnknownValue:
    return pddl.UNKNOWN

  assert False, "Unknown feature type: %s" % fval.__class__


def gen_fact_tuples(unions):
  for union in unions:
    if isinstance(union, specialentities.RelationUnion):
      try:
        source = feature_val_to_object(union.usource.alternativeValues[0])
        target = feature_val_to_object(union.utarget.alternativeValues[0])
      except Exception, e:
        print "Error getting source or target of relation %s." % union.entityID
        print "Message was: %s" % str(e)
        continue

      for feature in union.features:
        # choose feature val with highest probability:
        max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb)
        yield (feature.featlabel, source, target, feature_val_to_object(max_val))
        
    else:
      name = union.entityID
      object = pddl.TypedObject(name, pddl.t_object)
      
      for feature in union.features:
        # choose feature val with highest probability:
        max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb)
        yield (feature.featlabel, object, feature_val_to_object(max_val))

def filter_unknown_preds(fact_tuples):
  for ft in fact_tuples:
    feature_label = ft[0]
    if feature_label not in current_domain.functions and \
          feature_label not in current_domain.predicates:
      print "filtering feature assignment %s, because '%s' is not part of the planning domain" \
          % (map(str,ft), feature_label)
    else:
      #print "using", map(str, ft)
      yield ft

def tuples2facts(fact_tuples):
  for ftup in fact_tuples:
    feature_label = ftup[0]
    args = ftup[1:-1]
    val = ftup[-1]
    if feature_label in current_domain.functions:
      func = current_domain.functions.get(feature_label, args)
      yield state.Fact(state.StateVariable(func, args), val)
    else:
      assert feature_label in current_domain.predicates
      pred = current_domain.predicates.get(feature_label, args)
      yield state.Fact(state.StateVariable(pred, args), val)

def unify_objects(obj_descriptions):
  namedict = {}
  for ftup in obj_descriptions:
    feature_label = ftup[0]

    result = [feature_label]
    for obj in ftup[1:]:
      if obj.name in namedict:
        result.append(namedict[obj.name])
      else:
        namedict[obj.name] = obj
        result.append(obj)
        
    yield tuple(result)
      
def infer_types(obj_descriptions):
  constraints = defaultdict(set)
  for ftup in obj_descriptions:
    pred = ftup[0]
    args = ftup[1:]
    if pred in current_domain.functions:
      declarations = current_domain.functions[pred]
      is_function = True
    else:
      declarations = current_domain.predicates[pred]
      is_function = False
    for declaration in declarations:
      ftypes = map(lambda a: a.type, declaration.args)
      ftypes.append(declaration.type)

      #only consider this function if the basic value types (object, boolean, number) match
      if len(args) == len(ftypes) and all(map(lambda t, a: t.equal_or_subtype_of(a.type), ftypes, args)):
        for arg, type in zip(args, ftypes):
          constraints[arg].add(type)
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
      print "%s could be of types %s or %s" % (obj.name, type1, type2)
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
    print "goal:",problem.goal
  except pddl.parser.ParseError:
    problem.goal = pddl.conditions.Falsity()

  task._mapltask = problem
  task.set_state(state.State(facts, problem))
  
  return task  

def generate_mapl_state(task_desc, task):
  global current_domain
  current_domain = task._mapldomain
  
  obj_descriptions = list(unify_objects(filter_unknown_preds(gen_fact_tuples(task_desc.state))))
  
  objects = infer_types(obj_descriptions)
  task.namedict = rename_objects(objects)

  facts = list(tuples2facts(obj_descriptions))
  return objects, facts


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
  
