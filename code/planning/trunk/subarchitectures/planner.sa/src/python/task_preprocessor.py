from collections import defaultdict
from string import maketrans

from standalone.task import Task  # requires standalone planner to be in PYTHONPATH already
import standalone.mapl_new as mapl
import standalone.state_new as state
import binder.autogen.featvalues as featvalues

# Using string templates only until move to the proper ICE types
MAPL_TASK_TMPL = """
(define (problem cogxtask) (:domain cogx)
(:objects
%s
)
(:init
%s
)
(:goal (and
%s
)))
"""

forbidden_letters = "-:"
replace_chr = "_"
trans_tbl = maketrans(forbidden_letters, replace_chr * len(forbidden_letters))

# attention: current_domain is used as a global in this module
current_domain = None

def build_namedict(type_dict, tuples, known_constants):
  namedict = {}
  for obj in known_constants:
    namedict[obj] = obj
  for feature_label, union_name, val in tuples:
    for bname in [union_name, val]:
      if bname in namedict:
        continue
      thetype = type_dict[bname]
      pname = "%s_%s" % (thetype, bname.translate(trans_tbl))
      pname = pname.lower()
      namedict[bname] = pname
      namedict[pname] = bname  # for recovering the original name
  return namedict

def rename_objects(objects):
  namedict = {}
  for obj in objects:
    oldname = obj.name
    obj.name = "%s_%s" % (obj.type, obj.name.translate(trans_tbl))
    obj.name = obj.name.lower()
    namedict[oldname] = obj
    namedict[obj] = oldname
    
  return namedict

def feature_val_to_object(fval):
  if fval.__class__ == featvalues.StringValue:
    #lookup constants
    if fval.val in current_domain:
      return current_domain[fval.val]
    return mapl.types.TypedObject(fval.val, mapl.types.objectType)
  
  elif fval.__class__ == featvalues.AddressValue:
    #todo: how to support address values sensibly?
    return mapl.types.TypedObject(fval.val, mapl.types.objectType)
  
  elif fval.__class__ == featvalues.IntegerValue:
    return mapl.types.TypedObject(fval.val, mapl.types.numberType)
  
  elif fval.__class__ == featvalues.BooleanValue:
    if fval.val:
      return mapl.types.TRUE
    return mapl.types.FALSE
  
  elif fval.__class__ == featvalues.UnknownValue:
    return mapl.types.UNKNOWN

  assert False, "Unknown feature type: %s" % fval.__class__


def gen_fact_tuples(unions):
  for union in unions:
    #name = map_name(union.entityID, prefix="union")
    name = union.entityID
    object = mapl.types.TypedObject(name, mapl.types.objectType)
    for feature in union.features:
      # choose feature val with highest probability:
      max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb)
      #valname = map_name(max_val.val)
      #valname = max_val.val
      yield (feature.featlabel, object, feature_val_to_object(max_val))

def filter_unknown_preds(fact_tuples):
  for ft in fact_tuples:
    feature_label, union_object, val = ft
    if feature_label not in current_domain.functions and \
          feature_label not in current_domain.predicates:
      print "filtering feature assignment %s, because '%s' is not part of the planning domain" \
          % (map(str,ft), feature_label)
    else:
      yield ft

def tuples2strings(fact_tuples, nd):
  for feature_label, union_name, val in fact_tuples:
    if feature_label in current_domain.functions:
      yield "(= (%s %s) %s)" % (feature_label, nd[union_name], nd[val])
    else:
      assert feature_label in current_domain.predicates
      yield "(%s %s %s)" % (feature_label, nd[union_name], nd[val])

def tuples2facts(fact_tuples):
  for feature_label, union, val in fact_tuples:
    if feature_label in current_domain.functions:
      func = current_domain.functions.get(feature_label, [union])
      yield state.Fact(state.StateVariable(func, [union]), val)
    else:
      assert feature_label in current_domain.predicates
      pred = current_domain.predicates.get(feature_label, [union, val])
      yield state.Fact(state.StateVariable(pred, [union, val]), mapl.types.TRUE)

def unify_objects(obj_descriptions):
  namedict = {}
  for feature_label, union, val in obj_descriptions:
    if union.name in namedict:
      union = namedict[union.name]
    else:
      namedict[union.name] = union
      
    if val.name in namedict:
      val = namedict[val.name]
    else:
      namedict[val.name] = val
    yield feature_label, union, val
      
def infer_types(obj_descriptions):
  constraints = defaultdict(set)
  for pred, name, val in obj_descriptions:
    if pred in current_domain.functions:
      declarations = current_domain.functions[pred]
      is_function = True
    else:
      declarations = current_domain.predicates[pred]
      is_function = False
    for declaration in declarations:
      nametype = declaration.args[0].type
      valtype = declaration.type if is_function else declaration.args[1].type
      #only consider this function if the basic value types (object, boolean, number) match
      if nametype.equalOrSubtypeOf(name.type) and valtype.equalOrSubtypeOf(val.type):
        constraints[name].add(nametype)
        constraints[val].add(valtype)
        #print "Inferring: %s is instance of %s because of use in %s" % (name, nametype, declaration)
        #print "Inferring: %s is instance of %s because of use in %s" % (val, valtype, declaration)
        
  #type_dict = {}
  objects = set()
  for obj in constraints:
    if obj in (mapl.types.UNKNOWN, mapl.types.UNDEFINED) or obj in current_domain:
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
      elif type1.isSubtypeOf(type2):
        return -1
      elif type2.isSubtypeOf(type1):
        return 1
      assert False, "Multiple inheritance not supported yet"
    most_spec_type = sorted(types, cmp=type_cmp)[0]
    #type_dict[obj] = most_spec_type
    obj.type = most_spec_type
    objects.add(obj)
  #return type_dict
  return objects

def generate_mapl_task(task_desc, domain_fn):
  global current_domain
  task = Task()
  task.load_mapl_domain(domain_fn)
  current_domain = task._mapldomain
  
  #const_names = set(obj.name for obj in current_domain.constants)
  obj_descriptions = list(unify_objects(filter_unknown_preds(gen_fact_tuples(task_desc.state))))
  
  objects = infer_types(obj_descriptions)
  task.namedict = rename_objects(objects)

  facts = list(tuples2facts(obj_descriptions))
  init = [f.asLiteral(useEqual=True) for f in facts]

  problem = mapl.problem.Problem("cogxtask", objects, init, None, task._mapldomain)
  problem.goal = mapl.parser.Parser.parseAs(task_desc.goal.split("\n"), mapl.conditions.Condition, problem)

  task._mapltask = problem
  task.set_state(state.State(facts, problem))
  
  #current_domain.namedict = build_namedict(type_dict, obj_descriptions, const_names)
  #nd = current_domain.namedict
  #obj_declarations = "\n".join("%s - %s" % (nd[obj], type_dict[obj]) for obj in type_dict if obj not in const_names)
  #facts = "\n".join(tuples2strings(obj_descriptions, nd))
  #problem_str = MAPL_TASK_TMPL % (obj_declarations, facts, task_desc.goal)
  #print problem_str
  #task.parse_mapl_problem(problem_str)
  return task  

def generate_mapl_state(task_desc, task):
  global current_domain
  current_domain = task._mapldomain
  
  obj_descriptions = list(unify_objects(filter_unknown_preds(gen_fact_tuples(task_desc.state))))
  
  objects = infer_types(obj_descriptions)
  task.namedict = rename_objects(objects)

  facts = list(tuples2facts(obj_descriptions))
#   const_names = set(obj.name for obj in current_domain.constants)
#   obj_descriptions = list(filter_unknown_preds(gen_fact_tuples(task_desc.state)))
#   type_dict = infer_types(obj_descriptions)
#   current_domain.namedict = build_namedict(type_dict, obj_descriptions, const_names)
#   nd = current_domain.namedict

#   objects = build_objectdict(nd, type_dict)
#   facts = tuples2facts(obj_descriptions, objects)
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
  
