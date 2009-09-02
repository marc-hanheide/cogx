from collections import defaultdict

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


def union2name(union):
  return "union%s" % union.entityID

def union2type_declaration(union):
  """perform some basic type inference to determine the object type"""
  return "object"

def gen_fact_tuples(unions):
  for union in unions:
    name = union2name(union)
    for feature in union.features:
      # choose feature val with highest probability:
      max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb) 
      yield (feature.featlabel, name, max_val.val)

def filter_unknown_preds(fact_tuples, domain):
  for ft in fact_tuples:
    feature_label, union_name, val = ft
    if feature_label not in domain.functions:
      print "filtering feature assignment %s, because '%s' is not part of the planning domain" \
          % (ft, feature_label)
    else:
      yield ft

def tuples2strings(fact_tuples):
  for ft in fact_tuples:
    yield "(= (%s %s) %s)" % ft

def infer_types(obj_descriptions, domain):
    constraints = defaultdict(set)
    for pred, name, val in obj_descriptions:
        functions = domain.functions[pred]
        for function in functions:
            assert len(function.args) == 1
            arg0type = function.args[0].type
            valtype = function.type
            constraints[name].add(arg0type)
            constraints[val].add(valtype)
            print "Inferring: %s is instance of %s because of use in %s" % (name, arg0type, function)
            print "Inferring: %s is instance of %s because of use in %s" % (val, valtype, function)
    for obj in constraints:
        # now find most specific type
        if obj not in constraints:
            constraints[obj].add("object")
        types = constraints[obj]
        print "%s has the following type constraints %s" % (obj, map(str,types))
        def type_cmp(type1, type2):
            if type1.isSubtypeOf(type2):
                return -1
            elif type2.isSubtypeOf(type1):
                return 1
            assert False, "Multiple inheritance not supported yet"
        most_spec_type = sorted(types, cmp=type_cmp)[0]
        yield obj, most_spec_type

def generate_mapl_task(task_desc, domain_fn):
    task = Task()
    task.load_mapl_domain(domain_fn)
    domain = task._mapldomain

    obj_descriptions = list(filter_unknown_preds(gen_fact_tuples(task_desc.state), task._mapldomain))
    types = infer_types(obj_descriptions, domain)
    obj_declarations = "\n".join("%s - %s" % (obj, obj_type) for (obj, obj_type) in types)
    facts = "\n".join(tuples2strings(obj_descriptions))
    problem_str = MAPL_TASK_TMPL % (obj_declarations, facts, task_desc.goal)
    print problem_str
    task.parse_mapl_problem(problem_str)
    return task
