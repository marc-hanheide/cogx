from collections import defaultdict
from string import maketrans

from standalone.task import Task  # requires standalone planner to be in PYTHONPATH already

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
      namedict[bname] = pname
      namedict[pname] = bname  # for recovering the original name
  return namedict
      
def gen_fact_tuples(unions):
  for union in unions:
    #name = map_name(union.entityID, prefix="union")
    name = union.entityID
    for feature in union.features:
      # choose feature val with highest probability:
      max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb) 
      #valname = map_name(max_val.val)
      valname = max_val.val
      yield (feature.featlabel, name, valname)

def filter_unknown_preds(fact_tuples):
  for ft in fact_tuples:
    feature_label, union_name, val = ft
    if feature_label not in current_domain.functions and \
          feature_label not in current_domain.predicates:
      print "filtering feature assignment %s, because '%s' is not part of the planning domain" \
          % (ft, feature_label)
    else:
      yield ft

def tuples2strings(fact_tuples, nd):
  for feature_label, union_name, val in fact_tuples:
    if feature_label in current_domain.functions:
      yield "(= (%s %s) %s)" % (feature_label, nd[union_name], nd[val])
    else:
      assert feature_label in current_domain.predicates
      yield "(%s %s %s)" % (feature_label, nd[union_name], nd[val])

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
        constraints[name].add(nametype)
        constraints[val].add(valtype)
        print "Inferring: %s is instance of %s because of use in %s" % (name, nametype, declaration)
        print "Inferring: %s is instance of %s because of use in %s" % (val, valtype, declaration)
    type_dict = {}
    for obj in constraints:
      # now find most specific type
      if obj not in constraints:
        constraints[obj].add("object")
      types = constraints[obj]
      print "%s has the following type constraints %s" % (obj, map(str,types))
      def type_cmp(type1, type2):
        if type1 == type2:
          return 0
        elif type1.isSubtypeOf(type2):
          return -1
        elif type2.isSubtypeOf(type1):
          return 1
        assert False, "Multiple inheritance not supported yet"
      most_spec_type = sorted(types, cmp=type_cmp)[0]
      type_dict[obj] = most_spec_type
    return type_dict

def generate_mapl_task(task_desc, domain_fn):
    global current_domain
    task = Task()
    task.load_mapl_domain(domain_fn)
    current_domain = task._mapldomain
    const_names = set(obj.name for obj in current_domain.constants)
    obj_descriptions = list(filter_unknown_preds(gen_fact_tuples(task_desc.state)))
    type_dict = infer_types(obj_descriptions)
    current_domain.namedict = build_namedict(type_dict, obj_descriptions, const_names)
    nd = current_domain.namedict
    obj_declarations = "\n".join("%s - %s" % (nd[obj], type_dict[obj]) for obj in type_dict if obj not in const_names)
    facts = "\n".join(tuples2strings(obj_descriptions, nd))
    problem_str = MAPL_TASK_TMPL % (obj_declarations, facts, task_desc.goal)
    print problem_str
    task.parse_mapl_problem(problem_str)
    return task
