import os, sys, traceback, Ice
from os.path import abspath, dirname, join, isdir

import autogen.Planner as Planner
import binder.autogen.core
import cast.core

this_path = abspath(dirname(__file__))

def extend_pythonpath():
  """add standalone planner to PYTHONPATH"""
  standalone_path = join(this_path, "standalone")
  sys.path.insert(0, standalone_path)
extend_pythonpath()  

from standalone.task import Task
from standalone.planner import Planner as StandalonePlanner

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

TEST_DOMAIN_FN = join(dirname(__file__), "../../test_data/cp_test.domain.mapl")
TEST_TASK_FN = join(dirname(__file__), "../../test_data/cp_test.task.mapl")

def union2name(union):
  return "union%s" % union.entityID

def union2type_declaration(union):
  """perform some basic type inference to determine the object type"""
  return "object"

def gen_type_declarations(unions):
  for union in unions:
    yield "%s - %s" % (union2name(union), union2type_declaration(union))

def gen_fact_tuples(unions):
  for union in unions:
    name = union2name(union)
    for feature in union.features:
      # choose feature val with highest probability:
      max_val = max((val for val in feature.alternativeValues), key=lambda v: v.independentProb) 
      yield (feature.featlabel, name, max_val.val)

def filter_unknown_preds(fact_tuples, domain):
  for ft in fact_tuples:
    if pred not in domain.functions:
      feature_label, union_name, val = ft
      print "filtering feature assignment %s, because '%s' is not part of the planning domain" \
          % (ft, feature_label)
    else:
      yield ft

def tuples2strings(fact_tuples):
  for ft in fact_tuples:
    yield "(%s %s : %s)" % ft

class PythonServer(Planner.PythonServer, cast.core.CASTComponent):
  def __init__(self):
    self.client = None
    self.planner = StandalonePlanner()
    print "new PythonServer"

  def configure(self,config,current):
    pass

  def start(self,config):
    pass

  def stop(self,config):
    pass

  def runComponent(self):
    pass

  def registerTask(self, task_desc, current=None):
    # MB: id?
    print "Planner PythonServer: New PlanningTask received:"
    print "GOAL: " + task_desc.goal;
    #print "OBJECTS: " + task_desc.objects;
    #print "INIT: " + task_desc.state;

    task = Task()
    task.load_mapl_domain(TEST_DOMAIN_FN)
    
    

    #obj_descriptions = "\n".join(fact for union in task_desc.state for fact in union2facts(union))
    obj_descriptions = tuples2strings(filter_unknown_preds(gen_fact_tuples(unions), task._mapldomain))
    obj_descriptions = "\n".join(obj_descriptions)
    obj_declarations = "\n".join(gen_type_declarations(task_desc.state))
    problem_str = MAPL_TASK_TMPL % (obj_declarations, obj_descriptions, task_desc.goal)
    print problem_str


    #task.parse_mapl_problem(problem_str)
#     task.load_mapl_problem(TEST_TASK_FN)
    self.planner.register_task(task)

#     task.mark_changed()
#     task.activate_change_dectection()
#     plan = task.get_plan()
    
#     make_dot = True
#     if make_dot:
#       dot_str = plan.to_dot()
#       dot_fn = abspath(join(this_path, "plan.dot"))
#       print "Generating and showing plan in .dot format next.  If this doesn't work for you, edit show_dot.sh"
#       print "Dot file is stored in", dot_fn
#       show_dot_script = abspath(join(this_path, "../..", "show_dot.sh"))
#       open(dot_fn, "w").write(dot_str)
#       os.system("%s %s" % (show_dot_script, dot_fn)) 

#     if(self.client is None):
#       print "ERROR!!"

#     task_desc.plan = str(plan)
#     self.client.deliverPlan(task_desc);
    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    
  def registerClient(self, Client, current=None):
    print "Planner PythonServer: running"
    self.client = Client
