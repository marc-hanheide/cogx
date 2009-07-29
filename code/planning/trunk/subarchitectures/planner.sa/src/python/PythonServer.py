import sys, traceback, Ice

import autogen.Planner as Planner
import cast.core

# def extend_pythonpath():
#   from os.path import abspath, dirname, join, isdir
#   standalone_path = abspath(dirname(__file__))
# extend_pythonpath()  

# from standalone.task import Task
# from standalone.planner import StandalonePlanner

# # Using string templates only until move to the proper ICE types
# MAPL_TASK_TMPL = """
# (define (problem cosytask) (:domain coffee)
# (:objects
# %s
# )
# (:init
# %s
# )
# (:goal (and
# %s
# )))
# """

# TEST_DOMAIN_FN = ""



class PythonServerI(Planner.PythonServer, cast.core.CASTComponent):
  def __init__(self):
    self.client = None
    self.planner = StandalonePlanner()
    print "new PythonServer"

  def configure(self):
    pass

  def start(self):
    pass

  def stop(self):
    pass

  def runComponent(self):
    pass

  def registerTask(self, task_desc, current=None):
    # MB: id?
    print "Planner PythonServer: New PlanningTask received:"
    print "GOAL: " + task_desc.goal;
    print "OBJECTS: " + task_desc.objects;
    print "INIT: " + task_desc.state;

#     print sys.path
#     task = Task()
#     task.read_mapl_domain(TEST_DOMAIN_FN)
#     problem_str = MAPL_TASK_TMPL % (task_desc.objects, task_desc.state, task_desc.goal)
#     task.read_mapl_problem(problem_str)
#     self.planner.register_task(task)
#     task.set_planning_status(PlanningStatusEnum.TASK_CHANGED)
#     task.activate_change_dectection()


    task_desc.plan = "SOMEDAY HERE WILL BE A PLAN!!!"

    if(self.client is None):
      print "ERROR!!"

    self.client.deliverPlan(task_desc);
    # add task to some queue or start planning right away. when done call self.client.deliverPlan(string plan)
    
  def registerClient(self, Client, current=None):
    print "Planner PythonServer: running"
    self.client = Client
