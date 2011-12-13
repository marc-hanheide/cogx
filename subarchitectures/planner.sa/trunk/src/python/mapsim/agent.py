from standalone import pddl, plans
from standalone import statistics

from standalone.task import PlanningStatusEnum, Task
import standalone.globals as global_vars

from standalone import config
log = config.logger("mapsim")

statistics_defaults = dict(
    execution_time=0.0,
    failed_execution_attempts=0,
    physical_actions_executed=0,
    sensor_actions_executed=0,
    speech_acts_executed=0,
    total_plan_cost=0,
    reward=0
    )


task_id = 0
def next_id():
    global task_id
    task_id += 1
    return task_id-1

def loggingScope(f):
    def new_f(self, *args, **kwargs):
        oldscope = config.logfile_scope
        if global_vars.mapsim_config.separate_logs:
            config.logfile_scope = self.name
        rval = f(self, *args, **kwargs)
        config.logfile_scope = oldscope
        return rval
        
    new_f.__name__ = f.__name__
    return new_f

class BaseAgent(object):
    def __init__(self, simulator):
        self.simulator = simulator
        self.running = False

    def run(self):
        self.running = True
        self.failure_count = 0
        
    def execute(self, action, args):
        self.simulator.schedule(action, args, self)

    def done(self):
        self.running = False
        self.simulator.signal_done(self)

    def is_running(self):
        return self.running


class Agent(BaseAgent):
    def __init__(self, name, mapltask, planner, simulator):
        BaseAgent.__init__(self, simulator)
        
        self.name = name
        self.planner = planner
        self.statistics = statistics.Statistics(defaults = statistics_defaults)
        self.last_action = None

        self.new_task(mapltask)

    def get_state(self):
        return self.task.get_state()

    def update_state(self, svar, val):
        self.get_state()[svar] = val

    def new_task(self, mapltask):
        self.mapltask = mapltask.copy()

        prob_state = pddl.prob_state.ProbabilisticState.from_problem(self.mapltask)
        state = prob_state.determinized_state(0.1, 0.9)
        for svar, val in state.iteritems():
            if svar.modality == pddl.mapl.indomain:
                ivar = svar.as_modality(pddl.mapl.i_indomain, svar.modal_args)
                del state[svar]
                state[ivar] = val
        
        if global_vars.mapsim_config.add_assertions:
            self.task = Task(next_id(), self.mapltask, add_assertions=True)
            self.task.set_state(state)
        else:
            self.task = Task(next_id(), self.mapltask)
            self.task.set_state(state)
            
        self.planner.register_task(self.task)

    @loggingScope
    def run(self):
        BaseAgent.run(self)
        self.task.replan()
        self.execute_plan(self.task)

    def write_plan(self, plan):
        if global_vars.mapsim_config.write_dotfiles:
            G = plan.to_dot()
            G.write("plan.dot")
        if global_vars.mapsim_config.write_pdffiles:
            G = plan.to_dot() # a bug in pygraphviz causes write() to delete all node attributes when using subgraphs. So create a new graph.
            G.layout(prog='dot')
            G.draw("plan.pdf")
        
        
    @loggingScope
    @statistics.time_method_for_statistics("execution_time")
    def execute_plan(self, task):
        if task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
            return

        def is_pure_sensor(pnode):
            if isinstance(pnode.action, pddl.mapl.MAPLAction):
                return pnode.action.is_pure_sensor()
            return False

        def action_cmp(pnode1, pnode2):
            if is_pure_sensor(pnode1) and not is_pure_sensor(pnode2):
                return -1
            elif not is_pure_sensor(pnode1) and is_pure_sensor(pnode2):
                return 1

            for s1, s2 in [(pnode1.action.name, pnode2.action.name)] + zip(map(str, pnode1.args), map(str, pnode2.args)):
                if cmp(s1, s2) != 0:
                    return cmp(s1, s2)
            return 0
            
        plan = task.get_plan()
        self.write_plan(plan)
        #all_funcs = set(self.mapltask.functions) | set(self.mapltask.predicates)
        # print "instantiate:"
        #mapl.sas_translate.to_sas(self.mapltask)
        # print "executable:"
        # for a in self.mapltask.actions:
        #     a = pddl.mapl.MAPLObjectFluentNormalizer().translate(a, domain=self.mapltask)
    
        #     for c in pddl.sas_translate.instantiate_action(a, task.get_state(), all_funcs):
        #         print "(%s %s)" % (a.name, " ".join(o.name for o in c))

        executable = sorted(plan.executable(), cmp=action_cmp)
        log.info("executable actions: %s", " ".join(map(str, executable)))
        if executable:
            log.debug("trying to execute (%s %s)", executable[0].action.name, " ".join(map(str, executable[0].args)))
            self.last_action = executable[0]
            self.execute(executable[0].action.name, executable[0].full_args)
        else:
            log.debug("nothing more to do.")
            self.done()

  
    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        plan = self.task.get_plan()

        if plan is not None and action_status:
            self.last_action.status = action_status

        if action_status == plans.ActionStatusEnum.FAILED:
            if self.failure_count >= global_vars.mapsim_config.max_failures:
                return
            self.failure_count += 1
        else:
            self.failure_count = 0
            
        for f in new_facts:
            self.task.get_state().set(f)

        # print self.task.get_state()

        self.task.mark_changed()
        self.task.replan()
        self.execute_plan(self.task)

    def collect_statistics(self):
        """ return all stats collected by this agent (usually to the simulation) """
        return self.statistics.merge(self.task.statistics)
