import itertools, random, time
from collections import defaultdict

from standalone import pddl
from standalone import task, plans
from standalone import config
from standalone import statistics
from standalone.pddl import state, mapl

from standalone.planner import Planner as StandalonePlanner

import standalone.globals as global_vars

import agent

log = config.logger("mapsim")

statistics_defaults = dict(
    total_time=0.0,
    successful_runs=0,
    )


class Simulation(object):
    def __init__(self, scenario, runs=1, seed=None, agent_class = agent.Agent):
        self.planner = StandalonePlanner()
        self.scenario = scenario

        #create seed vector (one seed for each run)
        if not seed:
            seed = global_vars.mapsim_config.random
        if runs == 1:
            self.seeds = [seed]
        else:
            random.seed(seed)
            self.seeds = [hash(random.random()) for i in xrange(runs)]
            
        self.number_of_runs = runs
        self.domain = self.preprocess_domain(scenario.domain)
        #self.problem = self.preprocess_problem(scenario.world)
        self.problem = scenario.world
        
        self.stat_per_run = [None for i in xrange(runs)]
        self.statistics = statistics.Statistics(defaults=statistics_defaults)

        self.run_index = 0
        #self.prob_state = pddl.prob_state.ProbabilisticState.from_problem(self.problem)
        self.state =  pddl.prob_state.ProbabilisticState.sample_from_problem(self.problem, self.seeds[0])
        self.agents = {}
        for a, prob in scenario.agents.iteritems():
            if global_vars.mapsim_config.separate_logs:
                config.set_logfile("%s.log" % a, a)
                    
            self.agents[a] = agent_class(a, prob, self.planner, self)

    def reset(self, run):
        log.info("--------------------------------------------------------------------------------")
        log.info("Starting simulation run %d\n", run)
        self.time = 0
        self.queue = []
        self.run_index = run
        self.state =  pddl.prob_state.ProbabilisticState.sample_from_problem(self.problem, self.seeds[run])

        self.statistics.reset()
        self.planner.statistics.reset()
        
        for a in self.agents.itervalues():
            a.statistics.reset()
            a.new_task(self.scenario.agents[a.name])
            self.add_knowledge(a)

    def run(self):
        for i in xrange(self.number_of_runs):
            self.single_run(i)
            self.stat_per_run[self.run_index] = self.collect_statistics()
            print "Stats:", self.stat_per_run[self.run_index].sorted_repr(global_vars.mapsim_config.stat_order.split())
            log.info("stats: %s", repr(self.stat_per_run[self.run_index]))
        log.info("%d simulation runs completed", self.number_of_runs)

    @statistics.time_method_for_statistics("total_time")
    def single_run(self, run=0):
        self.reset(run)
        if 'partial-observability' in self.domain.requirements:
            log.debug("state:")
            log.debug(str(self.state))
            w = task.PDDLOutput(writer=pddl.dtpddl.DTPDDLWriter())
            w.write(self.problem, problem_fn="world.pddl")
        
        log.info("Starting simulation with random seed %d", self.seeds[self.run_index])
        for a in self.agents.itervalues():
            a.run()

        while any(a.is_running() for a in self.agents.itervalues()):
            if self.queue:
                self.time += 1
                action, args, agent = self.queue.pop()
                self.execute(action, args, agent)
            else:
                print "No agent is acting anymore. Aborting."
                return
            
        print "All agents are done."
        self.statistics.increase_stat("successful_runs")

    def add_knowledge(self, agent):
        """
        Translate (kval ?a ?svar) facts in the world state to actual knowledge in the agents state.
        """
        try:
            a = self.problem[agent.name]
        except KeyError, e:
            if agent.name == "default-agent":
                return
            raise e
        
        for svar, val in self.state.iteritems():
            if val == pddl.FALSE:
                continue
            if svar.modality == mapl.knowledge and svar.modal_args[0] == a:
                newvar = svar.nonmodal()
                agent.update_state(newvar, self.state[newvar])
        
    def preprocess_domain(self, dom):
        """
        Remove modal predicates from preconditions in the world domain.
        Remove assertions from the world domain.
        """
        
        def remove_visitor(cond, results=[]):
            if cond.__class__ == pddl.LiteralCondition:
                if cond.predicate in (mapl.knowledge, mapl.update, mapl.update_fail):
                    return pddl.conditions.Truth()
                if cond.predicate == mapl.indomain:
                    return pddl.conditions.LiteralCondition(pddl.equals, cond.args, cond.get_scope(), cond.negated) 
            if isinstance(cond, pddl.conditions.Conjunction):
                cond.parts = filter(lambda c: not isinstance(c, pddl.conditions.Truth) , results)
                if not cond.parts:
                    return pddl.conditions.Truth()
            elif isinstance(cond, pddl.conditions.Disjunction):
                if any(isinstance(c, pddl.conditions.Truth) for c in results):
                    return pddl.conditions.Truth()
                cond.parts = results
            elif isinstance(cond, pddl.conditions.QuantifiedCondition):
                if isinstance(results[0], pddl.conditions.Truth):
                    return pddl.conditions.Truth()
            return cond

        dom2 = dom.copy()

        dom2.set_actions(a for a in dom2.actions if a.replan is None)
        for a in dom2.actions:
            if a.precondition:
                a.precondition = a.precondition.visit(remove_visitor)
                
        self.observe_dict = defaultdict(set)
        if 'partial-observability' in dom2.requirements:
            for o in dom2.observe:
                if not o.execution:
                    for a in dom2.actions:
                        self.observe_dict[a.name].add(o)
                for ex in o.execution:
                    if not ex.negated:
                        self.observe_dict[ex.action.name].add(o)
                    else:
                        for a in dom2.actions:
                            if a != ex.action:
                                self.observe_dict[a.name].add(o)
                        
        return dom2

    def preprocess_problem(self, problem):
        from standalone import dt_problem
        pddl.dtpddl.DT2MAPLCompiler().translate(self.domain)
        dt_rules = pddl.translators.Translator.get_annotations(self.domain).get('dt_rules', [])
        
        prob_state = pddl.prob_state.ProbabilisticState.from_problem(problem)
        objects = self.domain.constants | problem.objects
        trees = dt_problem.StateTreeNode.create_root(prob_state, objects, dt_rules)
        hstate = dt_problem.HierarchicalState([], prob_state.problem)
        for t in trees:
            t.create_state(hstate)
        p_facts = hstate.init_facts()
        return pddl.Problem(problem.name, problem.objects, p_facts, None, self.domain)
        
        

    def schedule(self, action, args, agent):
        self.queue.append((action, args, agent))
        
    def execute(self, action, args, agent):
        action = self.domain.get_action(action)
        action.instantiate(args, self.problem)

        if isinstance(action, mapl.MAPLAction):
            agent_name = action.agents[0].get_instance().name
        else:
            agent_name = "default-agent"

        if agent != self.agents[agent_name]:
            print "%d: %s tried to execute action for %s (%s %s)" % (self.time, agent.name, agent_name, action.name, " ".join(a.name for a in args))
            log.debug("%d: %s tried to execute action for %s (%s %s)", self.time, agent.name, agent_name, action.name, " ".join(a.name for a in args))
            agent.statistics.increase_stat("failed_execution_attempts")
            agent.statistics.increase_stat("reward", 0.95**self.time * -global_vars.mapsim_config.reward)
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)
            return

        if self.state.is_executable(action):
            log.debug("%d: Agent %s executes (%s %s)", self.time, agent.name, action.name, " ".join(a.name for a in args))
            
            perceived_facts = []
            cterm = action.get_total_cost()
            cost = self.state.evaluate_term(cterm).value if cterm else 1
            agent.statistics.increase_stat("total_plan_cost", cost)
            agent.statistics.increase_stat("reward", 0.95**self.time * -cost)
            
            if action.effect:
                perceived_facts = self.execute_physical_action(action, agent)
                agent.statistics.increase_stat("physical_actions_executed")
            if isinstance(action, mapl.MAPLAction) and action.sensors:
                perceived_facts += self.execute_sensor_action(action, agent)
                agent.statistics.increase_stat("sensor_actions_executed")
            if self.observe_dict[action.name]:
                agent.statistics.increase_stat("sensor_actions_executed")
                perceived_facts += self.compute_observations(action, agent)

            log.debug("Facts sent to agent %s: %s", agent.name, ", ".join(map(str, perceived_facts)))
            action.uninstantiate()
            agent.updateTask(perceived_facts, plans.ActionStatusEnum.EXECUTED)
        else:
            print "%d: %s failed to execute (%s %s)" % (self.time, agent.name, action.name, " ".join(a.name for a in args))
            log.debug("%d: Agent %s failed to execute (%s %s)", self.time, agent.name, action.name, " ".join(a.name for a in args))
            agent.statistics.increase_stat("failed_execution_attempts")
            agent.statistics.increase_stat("reward", 0.95**self.time * -global_vars.mapsim_config.reward)
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)

    def execute_physical_action(self, action, agent):
        print "%d: %s executes (%s %s)" % (self.time, agent.name, action.name, " ".join(str(a.get_instance().name) for a in action.args))

        self.state.written_svars.clear()
        self.state.apply_effect(action.effect, trace_vars=True)
        self.state.clear_axiom_cache()
        
        new_facts = []
        for svar in self.state.written_svars:
            new_facts.append(state.Fact(svar, self.state[svar]))

        return new_facts

    def compute_observations(self, action, agent):
        obs = []
        for o in self.observe_dict[action.name]:
            det_args = None
            for ex in o.execution:
                if ex.action == action:
                    det_args = dict((oarg, aarg.get_instance()) for oarg, aarg in zip(ex.args, action.args ))
                    break

            assert det_args
                
            def get_objects(arg):
                if arg in det_args:
                    return [det_args[arg]]
                return list(self.problem.get_all_objects(arg.type))
            
            for mapping in o.smart_instantiate(o.get_inst_func(self.state), o.args, [get_objects(a) for a in o.args], self.problem):
                log.debug("%d: Agent %s executes observation (%s %s)", self.time, agent.name, o.name, " ".join(a.get_instance().name for a in o.args))
                facts = list(state.Fact.from_dict(self.state.get_effect_facts(o.effect)))
                log.debug("%d: Agent %s receives observations: %s", self.time, agent.name, ", ".join(map(str, facts)))
                for f in facts:
                    print "%d: %s observes: %s" % (self.time, agent.name, str(f.svar))
                obs += facts
        return obs

    def execute_sensor_action(self, action, agent):
        perceptions = []
        for se in action.sensors:
            svar = self.state.svar_from_term(se.get_term())
            if se.is_boolean():
                if isinstance(se.get_value(), pddl.predicates.FunctionTerm):
                    svar2 = state.StateVariable.from_term(se.get_value(), self.state)
                    value = self.state[svar2]
                else:
                    value = se.get_value().get_instance()

                if self.state[svar] == value:
                    print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), value.name)
                    perceptions.append(state.Fact(svar, value))
                else:
                    print "%d: %s senses %s != %s" % (self.time, agent.name, str(svar), value.name)
                    perceptions.append(state.Fact(svar.as_modality(mapl.i_indomain, [value]), pddl.FALSE))
            else:
                if self.state[svar] == pddl.UNKNOWN and svar.function.type == pddl.t_boolean:
                    #HACK: default to FALSE for undefined boolean fluents
                    print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), pddl.FALSE)
                    perceptions.append(state.Fact(svar, pddl.FALSE))
                else:
                    print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), self.state[svar].name)
                    perceptions.append(state.Fact(svar, self.state[svar]))
            
        return perceptions
                    

    def signal_done(self, agent):
        print "%d: Agent %s has reached its goal." % (self.time, agent.name)
        agent.statistics.increase_stat("reward", 0.95**self.time * global_vars.mapsim_config.reward)
        
    def collect_statistics(self):
        """ Collects statistics from agents and the planners used.
        Will aggregate most of the individual stats, e.g. sum up the
        number of plans generated or the planning times, but can also
        easily be extended to produce more individual statistics for
        each agent. """
        aggreg_funcs = defaultdict(lambda : sum)  # per default, sum up stats of the same name
        agent_stats = [agt.collect_statistics() for agt in self.agents.itervalues()]
        planner_stats = [self.planner.collect_statistics()]
        all_stats = agent_stats + planner_stats
        keys = set(k for stats in all_stats for k in stats)
        aggreg_vals = [(k,aggreg_funcs[k](stats[k] for stats in all_stats if k in stats)) for k in keys]
        total = self.statistics.merge(statistics.Statistics(aggreg_vals))
        averages = ["planning", "monitoring"]           # add some average runtimes
        for name in averages:
            calls = total[name+"_calls"]
            if calls:
                time = total[name+"_time"]
                total[name+"_avg_time"] = time / calls
            
        return total

    def collect_average_statistics(self):
        """ Collects statistics from the different planning runs.
        Will generate average values for interesting stats. """

        def avg_func(values):
            sum = 0
            count = 0
            for v in values:
                sum += v
                count +=1
                
            return float(sum)/count

        aggreg_funcs = defaultdict(lambda : avg_func)  # per default, calculate average
        aggreg_funcs["total_time"] = sum
        aggreg_funcs["successful_runs"] = sum

        all_stats = self.stat_per_run
        keys = set(k for stats in all_stats for k in stats)
        aggreg_vals = [(k,aggreg_funcs[k](stats[k] for stats in all_stats if k in stats)) for k in keys]
        total = statistics.Statistics(aggreg_vals)

        return total
    
