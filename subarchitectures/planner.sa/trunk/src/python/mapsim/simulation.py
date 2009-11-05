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

        self.remove_knowledge()

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

    def remove_knowledge(self):
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
            print "%d: Agent %s executes (%s %s)" % (self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args))) 
            log.debug("%d: Agent %s executes (%s %s)", self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args)))
            new_facts = set()
            for eff in action.effects:
                new_facts |= self.state.getEffectFacts(eff)
            percieved_facts = []
            
            if isinstance(action, mapl.sensors.Sensor):
                svar = state.StateVariable.fromTerm(action.get_term())
                if action.is_boolean():
                    value = action.get_value().getInstance()
                    if self.state[svar] == value:
                        percieved_facts = [state.Fact(svar, value)]
                    else:
                        percieved_facts = [state.Fact(svar.asModality(mapl.predicates.indomain, [value]), mapl.types.FALSE)]
                else:
                    percieved_facts = [state.Fact(svar, self.state[svar])]
                    
            else:
                for f in new_facts:
                    self.state.set(f)
                percieved_facts = list(new_facts)

            log.debug("Facts sent to agent %s: %s", agent.name, ", ".join(map(str, percieved_facts)))
            action.uninstantiate()
            agent.updateTask(percieved_facts, plans.ActionStatusEnum.EXECUTED)
        else:
            print "%d: Agent %s failed to execute (%s %s)" % (self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args))) 
            log.debug("%d: Agent %s failed to execute (%s %s)", self.time, agent.name, action.name, " ".join(map(lambda a: a.name, args)))
            action.uninstantiate()
            agent.updateTask([], plans.ActionStatusEnum.FAILED)

    def signal_done(self, agent):
        print "%d: Agent %s has reached its goal." % (self.time, agent.name)
        
