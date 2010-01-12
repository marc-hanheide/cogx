import itertools, random
from collections import defaultdict

from standalone import mapl_new as mapl
from standalone import plans
from standalone import config
from standalone import statistics
import standalone.mapl_new.state as state

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

import standalone.globals as global_vars

import agent

log = config.logger("mapsim")

statistics_defaults = dict(
    total_time=0.0,
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
        self.problem = scenario.world
        self.cleanup_actions()
        self.stat_per_run = [None for i in xrange(runs)]
        self.statistics = statistics.Statistics(defaults=statistics_defaults)

        self.run_index = 0
        self.state = state.State.fromProblem(self.scenario.world, seed=self.seeds[0])
        self.agents = {}
        for a, prob in scenario.agents.iteritems():
            #TODO better handling of non-strings in the configuration
            if global_vars.mapsim_config.separate_logs.lower() == "true":
                config.set_logfile("%s.log" % a, a)
                    
            self.agents[a] = agent_class(a, prob, self.planner, self)

    def reset(self, run):
        self.time = 0
        self.queue = []
        self.run_index = run
        self.state = state.State.fromProblem(self.scenario.world, seed=self.seeds[run])

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
            print "Stats:", self.stat_per_run[self.run_index]
        log.info("%d simulation runs completed", self.number_of_runs)

    @statistics.time_method_for_statistics("total_time")
    def single_run(self, run=0):
        self.reset(run)
        
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

    def add_knowledge(self, agent):
        """
        Translate (kval ?a ?svar) facts in the world state to actual knowledge in the agents state.
        """
        a = self.problem[agent.name]
        for svar, val in self.state.iteritems():
            if val == mapl.types.FALSE:
                continue
            if svar.modality == mapl.predicates.knowledge and svar.modal_args[0] == a:
                newvar = svar.nonmodal()
                agent.get_state()[newvar] = self.state[newvar]
        
    def cleanup_actions(self):
        """
        Remove modal predicates from preconditions in the world domain.
        Remove assertions from the world domain.
        """
        
        def remove_visitor(cond, results=[]):
            if cond.__class__ == mapl.conditions.LiteralCondition:
                if cond.predicate in (mapl.predicates.mapl_modal_predicates):
                    return None
                return cond
            if isinstance(cond, mapl.conditions.JunctionCondition):
                cond.parts = filter(None, results)
                if not cond.parts:
                    return None
                return cond
            if isinstance(cond, mapl.conditions.QuantifiedCondition):
                if results[0] is None:
                    return None
                return cond

        self.problem.actions = [a for a in self.problem.actions if a.replan is None]
        for a in itertools.chain(self.problem.actions, self.problem.sensors):
            if a.precondition:
                a.precondition.visit(remove_visitor)
            

    def schedule(self, action, args, agent):
        self.queue.append((action, args, agent))
        
    def execute(self, action, args, agent):
        action = self.problem.getAction(action)
        action.instantiate(args)

        if agent != self.agents[action.agents[0].getInstance().name]:
            other = action.agents[0].getInstance()
            print "%d: %s tried to execute action for %s (%s %s)" % (self.time, agent.name, other.name, action.name, " ".join(a.name for a in args))
            log.debug("%d: %s tried to execute action for %s (%s %s)", self.time, agent.name, other.name, action.name, " ".join(a.name for a in args))
            agent.statistics.increase_stat("failed_execution_attempts")
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)
            return
            
        if self.state.isExecutable(action):
            log.debug("%d: Agent %s executes (%s %s)", self.time, agent.name, action.name, " ".join(a.name for a in args))

            perceived_facts = []
            if isinstance(action, mapl.sensors.Sensor):
                perceived_facts = self.execute_sensor_action(action, agent)
                agent.statistics.increase_stat("sensor_actions_executed")
            else:
                perceived_facts = self.execute_physical_action(action, agent)
                agent.statistics.increase_stat("physical_actions_executed")

            log.debug("Facts sent to agent %s: %s", agent.name, ", ".join(map(str, perceived_facts)))
            action.uninstantiate()
            agent.updateTask(perceived_facts, plans.ActionStatusEnum.EXECUTED)
        else:
            print "%d: %s failed to execute (%s %s)" % (self.time, agent.name, action.name, " ".join(a.name for a in args))
            log.debug("%d: Agent %s failed to execute (%s %s)", self.time, agent.name, action.name, " ".join(a.name for a in args))
            agent.statistics.increase_stat("failed_execution_attempts")
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)

    def execute_physical_action(self, action, agent):
        print "%d: %s executes (%s %s)" % (self.time, agent.name, action.name, " ".join(a.getInstance().name for a in action.args))
        
        new_facts = self.state.getEffectFacts(action.effects)
        for f in new_facts:
            self.state.set(f)
            
        return list(new_facts)

    def execute_sensor_action(self, sensor, agent):
        svar = self.state.svarFromTerm(sensor.get_term())
        if sensor.is_boolean():
            value = sensor.get_value().getInstance()
            if self.state[svar] == value:
                print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), value.name)
                perception = state.Fact(svar, value)
            else:
                print "%d: %s senses %s != %s" % (self.time, agent.name, str(svar), value.name)
                perception = state.Fact(svar.asModality(mapl.predicates.i_indomain, [value]), mapl.types.FALSE)
        else:
            print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), self.state[svar].name)
            perception = state.Fact(svar, self.state[svar])
            
        return [perception]
                    

    def signal_done(self, agent):
        print "%d: Agent %s has reached its goal." % (self.time, agent.name)
        
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

        all_stats = self.stat_per_run
        keys = set(k for stats in all_stats for k in stats)
        aggreg_vals = [(k,aggreg_funcs[k](stats[k] for stats in all_stats if k in stats)) for k in keys]
        total = statistics.Statistics(aggreg_vals)

        return total
    
