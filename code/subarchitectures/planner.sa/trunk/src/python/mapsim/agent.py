from __future__ import absolute_import

from standalone import mapl_new as mapl
from standalone import state_new as state
from standalone import plans

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

task_id = 0
def next_id():
    global task_id
    task_id += 1
    return task_id-1

class BaseAgent(object):
    def __init__(self, simulator):
        self.simulator = simulator
        self.running = False

    def run(self):
        self.running = True
        
    def execute(self, action, args):
        self.simulator.schedule(action, args)

    def done(self):
        self.running = False
        self.simulator.signal_done(self)

    def is_running(self):
        return self.running

    
class Agent(BaseAgent):
    def __init__(self, name, mapltask, planner, simulator):
        BaseAgent.__init__(self, simulator)
        
        self.name = name
        self.mapltask = mapltask
        self.planner = planner
        
        self.task = Task(next_id(), mapltask)
        self.task.set_plan_callback(lambda task: self.getPlan(task))
        
        self.planner.register_task(self.task)

    def run(self):
        BaseAgent.run(self)
        self.task.replan()
        
    def getPlan(self, task):
        if task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
            return
            
        plan = task.get_plan()

        ordered_plan = plan.topological_sort(include_depths=False)
        outplan = []
        first_action = -1
        for i,pnode in enumerate(ordered_plan):
            if isinstance(pnode, plans.DummyNode) or not pnode.is_executable():
                continue
            if first_action == -1:
                first_action = i
      
            outplan.append(pnode)

        plan.execution_position = first_action
        if outplan:
            self.execute(outplan[0].action.name, outplan[0].full_args)
        else:
            self.done()

  
    def updateTask(self, new_facts, action_status=None):
        plan = self.task.get_plan()

        if plan is not None:
            executable_plan = plan.topological_sort(include_depths=False)[plan.execution_position:-1]
            if action_status:
                executable_plan[0].status = action_status
            
        for f in new_facts:
            self.task.get_state().set(f)
            
        self.task.mark_changed()
        self.task.replan()
