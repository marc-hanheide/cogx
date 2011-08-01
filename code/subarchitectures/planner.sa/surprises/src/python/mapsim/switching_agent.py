import os, re, time
from itertools import chain
from collections import defaultdict
from standalone import pddl, plans, task, statistics, planner, dt_problem, plan_postprocess, bayes, pstatenode
#from standalone import statistics

import standalone.globals as global_vars
import agent
from agent import loggingScope

from standalone import config
log = config.logger("switching")

sw_conf = global_vars.mapsim_config.switching
dt_conf = global_vars.config.dt

statistics_defaults = dict(
    dt_planning_calls=0,
    dt_actions_received=0,
    dt_planning_time=0.0,
    )

statistics_defaults.update(agent.statistics_defaults)


class StandaloneDTInterface(object):
    PDDL_REXP = re.compile("\((.*)\)")
    def __init__(self, dt_id, dt_task, action_callback, statistics):
        self.id = dt_id
        self.process = None
        self.action_callback = action_callback
        self.statistics = statistics
        
        planning_tmp_dir =  global_vars.config.tmp_dir
        tmp_dir = planner.get_planner_tempdir(planning_tmp_dir)
        self.domain_fn = os.path.join(tmp_dir, "domain%d.dtpddl" % self.id)
        self.problem_fn = os.path.join(tmp_dir, "problem%d.dtpddl" % self.id)

        #domain_out_fn = abspath(join(self.get_path(), "domain%d.dtpddl" % task.id))
        #w = standalone.task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
        #w.write(task.dt_task.problem, domain_fn=domain_out_fn)
      
        dt_task.write_dt_input(self.domain_fn, self.problem_fn)
        
    def run(self):
        self.statistics.increase_stat("dt_planning_calls")
        # import time
        # print "sleep"
        # time.sleep(1)

        exe = global_vars.config.dt.standalone_executable
        steps = global_vars.config.dt.steps
        istates = global_vars.config.dt.max_istates
        beta = global_vars.config.dt.beta
        
        import subprocess, atexit
        cmd = "%s --steps %d --max-iStates %d --beta %f --domain %s --problem %s" % (exe, steps, istates, beta, self.domain_fn, self.problem_fn)
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
        
    @statistics.time_method_for_statistics("dt_planning_time")
    def wait_for_action(self):
        while self.process.poll() is None:
            line = self.process.stdout.readline()
            log.debug("read: %s", line)
            if (line):
                self.statistics.increase_stat("dt_actions_received")
                log.debug("received action: %s", line)
                match = self.PDDL_REXP.search(line)
                if match:
                    alist = match.group(1).lower().strip().split(" ")
                    self.action_callback(alist[0], alist[1:])
                    return
        raise Exception("pcogx returned with exit code %d" % self.process.returncode)

    def send_observations(self, observations):
        print >> self.process.stdin, "O", " ".join(str(o) for o in observations)
        log.debug("sent observations: %s", " ".join(str(o) for o in observations))
        self.wait_for_action()
        
        
class SwitchingAgent(agent.Agent):
    def __init__(self, name, mapltask, planner, simulator):
        self.last_dt_id = 0
        self.dt_interface = None

        t = pddl.translators.RemoveTimeCompiler();
        mapltask = t.translate(mapltask)
        
        self.domain = mapltask.domain

        agent.Agent.__init__(self, name, mapltask, planner, simulator)
        self.statistics = statistics.Statistics(defaults = statistics_defaults)
        
    def new_task(self, mapltask):
        self.dt_problem = mapltask
        
        self.dt_task = None
        self.step = 0
        self.plan_history = []
        self.percepts = []
        self.dt_active = False
        self.fail_count = defaultdict(lambda: 0)
        self.dt_committments = {}
        self.dt_disconfirms = defaultdict(set)

        #hierarchical = self.simulator.preprocess_problem(mapltask)
        # w = task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
        w = dt_problem.DTPDDLOutput()
        w.write(mapltask, problem_fn="tree.pddl")
        
        pnodes = pstatenode.PNode.from_problem(mapltask)
        pnodes, det_lits = pstatenode.PNode.simplify_all(pnodes)
        mapltask.init += det_lits
        self.init_pnodes = pnodes
        self.pnodes = pnodes

        self.state = pddl.prob_state.ProbabilisticState.from_problem(mapltask)

        dt_compiler = pddl.dtpddl.DT2MAPLCompilerFD()
        self.prob_functions = dt_compiler.get_prob_functions(self.dt_problem)
        self.cp_domain = dt_compiler.translate(self.domain, prob_functions=self.prob_functions)
        self.unary_ops = None

        self.solution_prob = -1
        if dt_conf.enabled:
            self.compute_solution_likelihood()
        else:
            self.relevant_facts = {}
        
        self.regenerate_planning_domain()

        # w = dt_problem.FlatDTPDDLOutput(self.pnodes)
        # w.write(mapltask, problem_fn="flat.dtpddl")
        
        det_state = self.state.determinized_state(sw_conf.rejection_ratio, sw_conf.known_threshold)
                
        facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in det_state.iterfacts()]
        # facts += [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.compute_logps(det_state)]
        cp_problem = pddl.Problem(mapltask.name, mapltask.objects, facts , mapltask.goal, self.cp_domain)

        self.bayes = bayes.BayesianState(self.state, mapltask, pnodes, self.domain)
        self.bayes.evaluate()
        
        self.task = task.Task(agent.next_id(), cp_problem)
        self.task.wait_for_effects = False
        self.planner.register_task(self.task)

    def update_state(self, svar, val):
        self.unary_ops = None
        self.task.get_state()[svar] = val
        self.state[svar] = val

    def calc_probability(self, plan):
        uncertain_facts = set()
        cost = 0
        for pnode in plan.nodes_iter():
            for svar, val in pnode.original_preconds:
                if svar.modality == pddl.mapl.hyp:
                    uncertain_facts.add(pddl.state.Fact(svar.nonmodal(), svar.modal_args[0]))
            cost += pnode.cost

        p = 1.0
        for svar, val in uncertain_facts:
            if not self.state.is_det(svar):
                p *= self.state[svar][val]

        print "p:", p, "c:", cost
        
    @loggingScope
    def run(self):
        agent.BaseAgent.run(self)
        self.task.replan()
        self.process_cp_plan()

    # def compute_logps(self, st):
    #     import math
    #     pfuncs = set(f for f in self.domain.functions if "log-%s" % f.name in self.domain.functions)
    #     facts = []
    #     for svar, val in st.iteritems():
    #         if svar.function in pfuncs and val != pddl.UNKNOWN and val.is_instance_of(pddl.t_number) and val.value > 0:
    #             logfunc = self.domain.functions.get("log-%s" % svar.function.name, svar.args)
    #             logvar = pddl.state.StateVariable(logfunc, svar.args)
    #             logval = pddl.types.TypedNumber(max(-math.log(val.value, 2), 0.1))
    #             facts.append(pddl.state.Fact(logvar, logval))
    #     return facts

    def compute_solution_likelihood(self):
        from standalone import relaxed_exploration
        goal_action = pddl.Action("goal_action", [], self.dt_problem.goal, None, self.dt_problem)
        det_state = self.state.determinized_state(sw_conf.rejection_ratio, sw_conf.known_threshold)
        actions = [a for a in self.cp_domain.actions if not a.name.startswith("commit-selected")]

        t0 = time.time()
        if not self.unary_ops:
            self.unary_ops, applicable_ops = relaxed_exploration.instantiate(actions, set(), det_state, self.cp_domain, [(goal_action, {})])
        else:
            applicable_ops = relaxed_exploration.initialize_ops(self.unary_ops, det_state)
        print "total time for preparation: %.2f" % (time.time()-t0)
        t0 = time.time()
        
        rel, p = relaxed_exploration.prob_explore(self.unary_ops, applicable_ops, det_state, [(goal_action, [])], self.state)
        print "total time for exploration: %.2f" % (time.time()-t0)

        self.relevant_facts = rel
        self.solution_prob = p
        
        return p

    def confirmation_prob(self):
        p = 1.0
        for svar, val in self.dt_committments.iteritems():
            p *= self.state.prob(svar, val)
        return p

    def process_cp_plan(self):
        plan = self.get_plan()

        if plan is None:
            self.plan_history.append(plan)
            return

        cost = sum(pnode.cost for pnode in plan.nodes_iter())
        reward_p = self.confirmation_prob()
        exp_reward = dt_conf.total_reward * reward_p - (1-reward_p) * dt_conf.total_reward - cost
        if exp_reward < 0:
            print "c: %d, P(r): %.2f -> R = %.1f" % (cost, reward_p, exp_reward)
            return

        if dt_conf.enabled and not self.relevant_facts:
            self.compute_solution_likelihood()
        # p = self.compute_solution_likelihood()
        # if dt_conf.total_reward * p < cost:
        #     print "%.3f * %d < %d" % (p, dt_conf.total_reward, cost)
        #     return

        if "partial-observability" in self.domain.requirements:
            log.debug("creating dt task")
            self.dt_task = dt_problem.DTProblem(plan, self.pnodes, self.fail_count, self.prob_functions, self.relevant_facts, self.domain)

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

        if status == plans.ActionStatusEnum.FAILED:
            return

        if plan is not None:
            pnode = plan.topological_sort()[plan.execution_position]
            pnode.status = status

        self.task.mark_changed()
        self.monitor_cp()

    def monitor_dt(self):
        #test if the dt goals are satisfied:
        sat = True
        for pnode in self.dt_task.subplan_actions:
            children = self.get_plan().successors(pnode, link_type="depends")
            if not all(self.task.planner.check_node(c, self.state) for c in children):
            #if any(fact not in self.state for fact in pnode.effects):
                sat = False
                break
        if len(self.dt_task.dt_plan) > 20:
            log.debug("More than 20 actions from dt. Force cancellation.")
            self.dt_done()
            return
        # if sat:
        #     log.debug("dt planning stopped. Subgoal reached.")
        #     self.dt_done()
        #     return

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
            log.debug("No observations from %s", str(dt_pnode))
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

        log.debug("dt planning cancelled.")
        self.dt_interface.kill()
        self.dt_interface = None
        self.dt_active = False

        if not self.dt_task.dt_plan:
            log.debug("dt plan returned without action.")
            for goal in self.dt_task.goals:
                self.fail_count[goal] += 1
                if self.fail_count[goal] > 4:
                    return
        else:
            for goal in self.dt_task.goals:
                if goal in self.fail_count:
                    del self.fail_count[goal]
        
        self.get_plan().execution_position = last_dt_action
        self.plan_history.append(self.dt_task)
        self.dt_task = None

        self.regenerate_planning_domain()
        self.update_planning_state()
        self.compute_solution_likelihood()
        
        self.task.set_plan(None)
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
            conf, disconf = self.dt_task.get_dt_results(pddl_action)
            # self.dt_disconfirms.clear()
            if conf and dt_conf.commitment_mode in ("conf", "all"):
                p = self.state.prob(conf.svar, conf.value)
                if p < 0.7:
                    print "Committment %s rejected (p=%.2f)" % (str(conf), p)
                    log.debug("Comittment %s rejected (p=%.2f)", str(conf), p)
                else:
                    print "Committed to:", conf, p
                    log.debug("DT committed to %s", str(conf))
                    self.dt_committments[conf.svar] = conf.value
            elif conf:
                p = self.state.prob(conf.svar, conf.value)
                print "Committed to:", conf, p

            if disconf and dt_conf.commitment_mode in ("disconfirm", "all"):
                p = self.state.prob(disconf.svar, disconf.value)
                if p > 0.2:
                    print "Disconfirmation %s rejected (p=%.2f)" % (str(disconf), p)
                    log.debug("Disconfirmation %s rejected (p=%.2f)", str(disconf), p)
                else:
                    print "Disconfirmed:", disconf, p
                    log.debug("DT disconfirmed %s", str(disconf))
                    self.dt_disconfirms[disconf.svar].add(disconf.value)
            elif disconf:
                p = self.state.prob(disconf.svar, disconf.value)
                print "Disconfirmed:", disconf, p

            if pddl_action.name == "cancel":
                log.warning("DT canceled without result")
                #return
                
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
            log.debug("First action: %s", str(nodes[0]))
            nodes[0].status = plans.ActionStatusEnum.IN_PROGRESS
            self.execute(nodes[0].action.name, nodes[0].full_args)
        else:
            log.debug("nothing more to do.")
            self.done()
                
    def dt_planning_active(self):
        if not dt_conf.enabled:
            return False
        
        plan = self.get_plan()
        if not plan or not self.dt_task:
            return False
        
        return self.dt_task.subplan_active(plan)

    def start_dt_planning(self):
        log.info("starting dt planner.")
        self.dt_active = True
        self.dt_interface = StandaloneDTInterface(self.last_dt_id, self.dt_task, lambda n,a: self.action_delivered(n,a), self.statistics)
        self.dt_interface.run()

    def deliverObservation(self, obs):
        rawobs = []
        for svar in obs:
            if svar is None:
                rawobs.append("(null)")
                continue

            if not self.dt_task.expected_observation(svar):
                log.info("observation %s was rejected by dt task.", str(svar))
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

    def update_planning_state(self):
        mod_state = self.state.copy()
        for svar, val in self.dt_committments.iteritems():
            mod_state[svar] = val
        for svar, vals in self.dt_disconfirms.iteritems():
            for val in vals:
                if not mod_state.is_det(svar):
                    mod_state[svar][val] = 0.0
                    # print svar, mod_state[svar]

        det_state = mod_state.determinized_state(sw_conf.rejection_ratio, sw_conf.known_threshold)
                
        facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in det_state.iterfacts()]
        # facts += [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.compute_logps(det_state)]
        cp_problem = pddl.Problem(self.dt_problem.name, self.dt_problem.objects, facts , self.dt_problem.goal, self.cp_domain)
        
        self.task.mapltask = cp_problem
        self.task.create_initial_state()

    def regenerate_planning_domain(self):
        def node_filter(node, val, p, facts):
            """ Returns two boolean values:
                First indicates that the node should always be included
                Second indicated that the node should be included if it has children"""
            for svar, v in chain([(node.svar, val)], facts.iteritems()):
                if svar.function == pddl.dtpddl.selected:
                    continue
                if svar in self.state and self.state.is_det(svar):
                    if self.state[svar] != v:
                        return False, False
                    continue
                if v in self.dt_disconfirms[svar]:
                    # print "reject:", node, val, " -- ", svar, v
                    return False, False
                if svar in self.relevant_facts and v not in self.relevant_facts[svar]:
                    # print "reject", node, svar, v
                    return False, False
                if self.relevant_facts and svar not in self.relevant_facts:
                    # print "reject", svar
                    return False, True
                if self.state[svar][v] < sw_conf.rejection_ratio:
                    highest = max(self.state[svar].itervalues())
                    if self.state[svar][v] < highest * sw_conf.rejection_ratio:
                        # print "reject:", node, self.state[svar][v]
                        return False, False
            return True, True
            
        dt_compiler = pddl.dtpddl.DT2MAPLCompilerFD()
        self.prob_functions = dt_compiler.get_prob_functions(self.dt_problem)
        self.cp_domain = dt_compiler.translate(self.domain, prob_functions=self.prob_functions)

        actions = []
        for n in self.pnodes:
            n.prepare_actions()
        for n in self.pnodes:
            actions += n.to_actions(self.cp_domain, filter_func=node_filter)

        for a in actions:
            self.cp_domain.add_action(a)
        
        
    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        new_percepts = []
        for f in new_facts:
            if f.svar.modality == pddl.dtpddl.observed or f.svar.function in self.domain.percepts:
                new_percepts.append(f.svar)
            else:
                self.state.set(f)
        self.percepts += new_percepts

        action = self.last_action.action
        action.instantiate(self.last_action.full_args, self.task.mapltask)
        if self.bayes.handle_obs(action, new_percepts):
            # def node_filter(node):
            #     for f in node.all_facts():
            #         if f.svar.function == pddl.dtpddl.selected:
            #             continue
            #         if f.svar not in self.state or not self.state.is_det(f.svar):
            #             return True
            #     return False

            result, node_probs = self.bayes.evaluate()
            self.pnodes = self.bayes.new_ptree(self.init_pnodes, node_probs)
            self.regenerate_planning_domain()

            for svar, dist in result.iteritems():
                if svar.function != pddl.dtpddl.selected and (svar not in self.state or not self.state.is_det(svar)):
                    self.state[svar] = dist
                    
            self.bayes.init(self.state, self.pnodes)
        
        action.uninstantiate()
        
        self.update_planning_state()
        # import time
        # print "sleep"
        # time.sleep(1)
                
        #TODO: start execution
        if self.dt_active:
            self.action_executed_dt(action_status)
        else:
            self.action_executed_cp(action_status)


class RandomAgent(SwitchingAgent):
    def run(self):
        agent.BaseAgent.run(self)
        action, args = self.choose_random_action()
        self.last_action = (action, args)
        self.count = 1
        self.execute(action.name, args)

    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        if self.count > 1:
            return

        new_percepts = []
        for f in new_facts:
            if f.svar.modality == pddl.dtpddl.observed or f.svar.function in self.domain.percepts:
                new_percepts.append(f.svar)
            else:
                self.state.set(f)
        self.percepts += new_percepts

        action = self.last_action[0]
        action.instantiate(self.last_action[1], self.task.mapltask)
        if self.bayes.handle_obs(action, new_percepts):
            # def node_filter(node):
            #     for f in node.all_facts():
            #         if f.svar.function == pddl.dtpddl.selected:
            #             continue
            #         if f.svar not in self.state or not self.state.is_det(f.svar):
            #             return True
            #     return False

            result, node_probs = self.bayes.evaluate()
            self.pnodes = self.bayes.new_ptree(self.init_pnodes, node_probs)
            self.regenerate_planning_domain()

            for svar, dist in result.iteritems():
                if svar.function != pddl.dtpddl.selected and (svar not in self.state or not self.state.is_det(svar)):
                    self.state[svar] = dist
                    
            self.bayes.init(self.state, self.pnodes)

        
        action.uninstantiate()
        self.update_planning_state()

        if self.task.get_state().is_satisfied(self.task.mapltask.goal):
            self.done()
        
        action, args = self.choose_random_action()
        self.last_action = (action, args)
        self.count += 1
        self.execute(action.name, args)
    

    def choose_random_action(self):
        det_state = self.state.determinized_state(sw_conf.rejection_ratio, sw_conf.known_threshold)
        prob = self.task.mapltask
        actions = []
        for action in self.cp_domain.actions:
            alist = [list(prob.get_all_objects(a.type)) for a in action.args]
            for mapping in action.smart_instantiate(action.get_inst_func(det_state), action.args, alist, prob):
                actions.append((action, [mapping[a] for a in action.args]))
        
        import random
        random.seed()
        return random.sample(actions, 1)[0]
        
            
