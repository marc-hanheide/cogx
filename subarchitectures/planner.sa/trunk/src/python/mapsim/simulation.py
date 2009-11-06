import itertools

from standalone import mapl_new as mapl
from standalone import state_new as state
from standalone import plans
from standalone import config

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

import agent
log = config.logger("mapsim")

class Simulation(object):
    def __init__(self, scenario):
        self.planner = StandalonePlanner()
        self.scenario = scenario
        self.state = state.State.fromProblem(scenario.world)
        self.problem = scenario.world

        self.agents = {}
        for a, prob in scenario.agents.iteritems():
            self.agents[a] = agent.Agent(a, prob, self.planner, self)

        self.queue = []
        self.time = 0

        self.cleanup_actions()
        for a in self.agents.itervalues():
            self.add_knowledge(a)

    def run(self):
        for a in self.agents.itervalues():
            a.run()

        while any(map(lambda a: a.is_running(), self.agents.itervalues())):
            if self.queue:
                self.time += 1
                action, args = self.queue.pop()
                self.execute(action, args)
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
                agent.getState()[newvar] = self.state[newvar]
        
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

        self.problem.actions = filter(lambda a: a.replan is None, self.problem.actions)
        for a in itertools.chain(self.problem.actions, self.problem.sensors):
            if a.precondition:
                a.precondition.visit(remove_visitor)
            

    def schedule(self, action, args):
        self.queue.append((action, args))
        
    def execute(self, action, args):
        action = self.problem.getAction(action)
        action.instantiate(args)
        
        agent = self.agents[action.agents[0].getInstance().name]

        if self.state.isExecutable(action):
            log.debug("%d: Agent %s executes (%s %s)", self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args)))

            percieved_facts = []
            if isinstance(action, mapl.sensors.Sensor):
                percieved_facts = self.execute_sensor_action(action, agent)
            else:
                percieved_facts = self.execute_physical_action(action, agent)

            log.debug("Facts sent to agent %s: %s", agent.name, ", ".join(map(str, percieved_facts)))
            action.uninstantiate()
            agent.updateTask(percieved_facts, plans.ActionStatusEnum.EXECUTED)
        else:
            print "%d: %s failed to execute (%s %s)" % (self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args))) 
            log.debug("%d: Agent %s failed to execute (%s %s)", self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args)))
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)

    def execute_physical_action(self, action, agent):
        print "%d: %s executes (%s %s)" % (self.time, agent.name, action.name, " ".join(map(lambda a: a.getInstance().name, action.args)))
        
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
                perception = state.Fact(svar.asModality(mapl.predicates.indomain, [value]), mapl.types.FALSE)
        else:
            print "%d: %s senses %s = %s" % (self.time, agent.name, str(svar), self.state[svar].name)
            perception = state.Fact(svar, self.state[svar])
            
        return [perception]
                    

    def signal_done(self, agent):
        print "%d: Agent %s has reached its goal." % (self.time, agent.name)
        
