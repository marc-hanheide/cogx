import os, re

from standalone import pddl, plans, task, planner, dt_problem, plan_postprocess, bayes
#from standalone import statistics

import standalone.globals as global_vars
import agent
from agent import loggingScope

from standalone import config
log = config.logger("switching")


class StandaloneDTInterface(object):
    PDDL_REXP = re.compile("\((.*)\)")
    def __init__(self, dt_id, dt_task, action_callback):
        self.id = dt_id
        self.process = None
        self.action_callback = action_callback
        
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = planner.get_planner_tempdir(planning_tmp_dir)
        self.domain_fn = os.path.join(tmp_dir, "domain%d.dtpddl" % self.id)
        self.problem_fn = os.path.join(tmp_dir, "problem%d.dtpddl" % self.id)

        #domain_out_fn = abspath(join(self.get_path(), "domain%d.dtpddl" % task.id))
        #w = standalone.task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
        #w.write(task.dt_task.problem, domain_fn=domain_out_fn)
      
        dt_task.write_dt_input(self.domain_fn, self.problem_fn)
        
    def run(self):
        import subprocess, atexit
        cmd = "%s --domain %s --problem %s" % (global_vars.config.dt.standalone_executable, self.domain_fn, self.problem_fn)
        log.debug("running dt planner with '%s'", cmd)
        self.process = subprocess.Popen(cmd.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=open("dtout.log", "w"))
        atexit.register(lambda: self.kill())
        log.debug("process %d created", self.process.pid)
        self.wait_for_action()

    def kill(self):
        if not self.process or self.process.returncode is not None:
            return
        log.debug("killing process %d", self.process.pid)
        self.process.terminate()
        self.process = None
        
    def wait_for_action(self):
        while True:
            line = self.process.stdout.readline()
            log.debug("read: %s", line)
            if (line):
                log.debug("received action: %s", line)
                match = self.PDDL_REXP.search(line)
                if match:
                    alist = match.group(1).lower().strip().split(" ")
                    self.action_callback(alist[0], alist[1:])
                    return

    def send_observations(self, observations):
        print >> self.process.stdin, "O", " ".join(str(o) for o in observations)
        log.debug("sent observations: %s", " ".join(str(o) for o in observations))
        self.wait_for_action()
                            

class SwitchingAgent(agent.Agent):
    def __init__(self, name, mapltask, planner, simulator):
        self.last_dt_id = 0
        self.dt_interface = None
        
        if global_vars.config.base_planner.name == "TFD":
            dt_compiler = pddl.dtpddl.DT2MAPLCompiler 
        elif global_vars.config.base_planner.name == "ProbDownward":
            dt_compiler = pddl.dtpddl.DT2MAPLCompilerFD
        else:
            assert False, "Only TFD and modified Fast Downward (ProbDownward) are supported"
        
        self.domain = mapltask.domain
        self.cp_domain = dt_compiler().translate(mapltask.domain)
        
        agent.Agent.__init__(self, name, mapltask, planner, simulator)
        
    def new_task(self, mapltask):
        self.dt_problem = mapltask
        
        self.dt_task = None
        self.step = 0
        self.plan_history = []
        self.percepts = []
        self.dt_active = False

        self.state = pddl.prob_state.ProbabilisticState.from_problem(mapltask)
        det_state = self.state.determinized_state(0.05, 0.95)
        facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in det_state.iterfacts()]
        facts += [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.compute_logps(det_state)]
        cp_problem = pddl.Problem(mapltask.name, mapltask.objects, facts , mapltask.goal, self.cp_domain)

        self.bayes = bayes.BayesianState(self.state, mapltask, self.domain)
        #bnetif = BnetInterface(self.bayes.nodes.values(), self.bayes.edges, debug=False)
        #result = bnetif.evaluate()
        #for node, values in result.iteritems():
        #    print node.var, values
        self.bayes.evaluate()
        
        self.task = task.Task(agent.next_id(), cp_problem)
        self.task.wait_for_effects = False
        self.planner.register_task(self.task)

    @loggingScope
    def run(self):
        agent.BaseAgent.run(self)
        self.task.replan()
        self.process_cp_plan()

    def compute_logps(self, st):
        import math
        pfuncs = set(f for f in self.domain.functions if "log-%s" % f.name in self.domain.functions)
        facts = []
        for svar, val in st.iteritems():
            if svar.function in pfuncs and val != pddl.UNKNOWN and val.is_instance_of(pddl.t_number) and val.value > 0:
                logfunc = self.domain.functions.get("log-%s" % svar.function.name, svar.args)
                logvar = pddl.state.StateVariable(logfunc, svar.args)
                logval = pddl.types.TypedNumber(max(-math.log(val.value, 2), 0.1))
                facts.append(pddl.state.Fact(logvar, logval))
        return facts
        
    def process_cp_plan(self):
        plan = self.get_plan()

        if plan is None:
            self.plan_history.append(plan)
            return

        if "partial-observability" in self.domain.requirements:
            log.info("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.domain)

            for pnode in plan.nodes_iter():
                if pnode.is_virtual():
                    pnode.status = plans.ActionStatusEnum.EXECUTED
            
            if self.dt_planning_active():
                self.dt_task.initialize(self.state)
                self.start_dt_planning()
                return
            
        self.write_plan(self.get_plan())

        ordered_plan = plan.topological_sort()
        exec_plan = []
        first_action = -1

        for i, pnode in enumerate(ordered_plan):
            if isinstance(pnode, plans.DummyNode) or not pnode.is_executable():
                continue
            if first_action == -1:
                first_action = i
            exec_plan.append(pnode)
            
        plan.execution_position = first_action
        self.dispatch_actions(exec_plan)

    def action_executed_dt(self, status):
        dt_pnode = self.dt_task.dt_plan[-1]

        #Failed dt action causes all actions in the subplan to fail
        dt_pnode.status = status
        if status == plans.ActionStatusEnum.FAILED:
            for pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.FAILED
            self.task.mark_changed()
            self.monitor_cp()
            return

        self.monitor_dt()
            
    def action_executed_cp(self, status):
        plan = self.get_plan()

        if plan is not None:
            pnode = plan.topological_sort()[plan.execution_position]
            pnode.status = status

        self.task.mark_changed()
        self.monitor_cp()

    def monitor_dt(self):
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            if any(fact not in self.state for fact in pnode.effects):
                sat = False
                break
        if sat:
            self.dt_done()

        dt_pnode = self.dt_task.dt_plan[-1]

        if self.dt_task.replanning_neccessary(self.state):
            log.info("DT task requires replanning")
            #send empty observations to terminate previous task
            self.deliverObservation([])
            self.dt_task.recompute_problem(self.state)
            #self.update_status(TaskStateEnum.WAITING_FOR_DT)
            self.start_dt_planning()

        def get_observations():
            result = []
            for svar in self.percepts:
                result.append(svar)
                log.info("delivered observation %s", str(svar))
            return result
            
        observations = get_observations()
        if not observations:
            log.info("No observations from %s", str(dt_pnode))
            observations.append(None)

        log.debug("delivered observations")
        self.deliverObservation(observations)
  
    def monitor_cp(self):
        # if self.dt_planning_active():
        #     self.process_cp_plan()
        #     return

        plan = self.task.get_plan()
        
        self.step += 1
        self.task.replan()
        if self.task.get_plan() != plan:
            self.plan_history.append(plan)
            
        self.process_cp_plan()

    def dt_done(self):
        last_dt_action = -1
        dt_action_found = False
        for i,pnode in enumerate(self.get_plan().topological_sort()):
            if pnode in self.dt_task.subplan_actions:
                pnode.status = plans.ActionStatusEnum.EXECUTED
                dt_action_found = True
                last_dt_action = i
            elif dt_action_found:
                break

        log.info("dt planning cancelled.")
        self.dt_interface.kill()
        self.dt_interface = None
        self.dt_active = False
            
        self.get_plan().execution_position = last_dt_action
        self.plan_history.append(self.dt_task)

        self.task.mark_changed()
        self.monitor_cp()

    def action_delivered(self, name, arguments):
        log.debug("raw action: (%s %s)", name, " ".join(arguments))
        args = [self.task.mapltask[a] for a in arguments]
        pddl_action = self.dt_task.dtdomain.get_action(name)

        log.debug("got action from DT: (%s %s)", name, " ".join(arguments))
        #log.debug("state is: %s", self.task.get_state())

        if pddl_action.name in set(a.name for a in self.dt_task.goal_actions):
            log.info("Goal action recieved. DT task completed")
            self.dt_done()
            return
        
        #TODO: using the last CP state might be problematic
        state = self.task.get_state().copy()
        try:
            pnode = plan_postprocess.getRWDescription(pddl_action, args, state, 1)
        except:
            log.error("Action (%s %s) not executable.", pddl_action.name, " ".join(arguments))
            return
            
        self.percepts = []
        self.dt_task.dt_plan.append(pnode)
        self.dispatch_actions([pnode])
        

    def dispatch_actions(self, nodes):
        if nodes:
            self.last_action = nodes[0]
            log.info("First action: %s", str(nodes[0]))
            nodes[0].status = plans.ActionStatusEnum.IN_PROGRESS
            self.execute(nodes[0].action.name, nodes[0].full_args)
        else:
            log.debug("nothing more to do.")
            self.done()
                
    def dt_planning_active(self):
        plan = self.get_plan()
        if not plan or not self.dt_task:
            return False
        
        return self.dt_task.subplan_active(plan)

    def start_dt_planning(self):
        log.debug("starting dt planner...")
        self.dt_active = True
        self.dt_interface = StandaloneDTInterface(self.last_dt_id, self.dt_task, lambda n,a: self.action_delivered(n,a))
        self.dt_interface.run()

    def deliverObservation(self, obs):
        rawobs = []
        for svar in obs:
            if svar is None:
                rawobs.append("(null)")
                continue
            
            if svar.modality:
                name = "%s-%s" % (svar.modality.name, svar.function.name)
            else:
                name = svar.function.name

            args = []
            for a in svar.get_args():
                if isinstance(a, pddl.TypedObject):
                    args.append(a)
                elif isinstance(a, pddl.FunctionTerm):
                    args += [a.object for a in a.args]
                
            rawobs.append("(%s %s)" % (name, " ".join(a.name for a in args)))

        self.dt_interface.send_observations(rawobs)
    
    def get_plan(self):
        plan = self.task.get_plan()

        if plan is None or self.task.planning_status == task.PlanningStatusEnum.PLANNING_FAILURE:
            return None

        return plan
        
    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        new_percepts = []
        for f in new_facts:
            if f.svar.modality == pddl.dtpddl.observed:
                new_percepts.append(f.svar)
            else:
                self.state.set(f)
        self.percepts += new_percepts

        action = self.last_action.action
        action.instantiate(self.last_action.full_args, self.task.mapltask)
        self.bayes.handle_obs(action, new_percepts)
        action.uninstantiate()
        result = self.bayes.evaluate()
        for node, probs in result.iteritems():
            if isinstance(node.var, str):
                continue # observation node
            svar = node.var

            dist = pddl.prob_state.ValueDistribution(dict((v,p) for v,p in zip(node.values, probs)))
            self.state[svar] = dist

        det_state = self.state.determinized_state(0.05, 0.95)
        facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in det_state.iterfacts()]
        facts += [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.compute_logps(det_state)]
        cp_problem = pddl.Problem(self.dt_problem.name, self.dt_problem.objects, facts , self.dt_problem.goal, self.cp_domain)
        
        self.task.mapltask = cp_problem
        self.task.create_initial_state()
                
        #TODO: start execution
        if self.dt_active:
            self.action_executed_dt(action_status)
        else:
            self.action_executed_cp(action_status)
