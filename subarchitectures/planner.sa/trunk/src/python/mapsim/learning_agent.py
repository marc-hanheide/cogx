import random

from standalone import mapl_new as mapl
from standalone import state_new as state
from standalone import plans
from standalone import statistics
from standalone import assertions, macros

from standalone.task import PlanningStatusEnum, Task
from standalone.planner import Planner as StandalonePlanner

import standalone.globals as global_vars

import agent, simulation
from agent import Agent, loggingScope
from simulation import Simulation

from standalone import config
log = config.logger("mapsim")

statistics_defaults = dict(
    failed_execution_attempts=0,
    physical_actions_executed=0,
    sensor_actions_executed=0,
    speech_acts_executed=0,
    learning_time=0,
    clustering_time=0
    )


alpha = 0.3

class LearningAgent(Agent):
    def __init__(self, name, mapltask, planner, simulator):       
        self.macros = []
        self.name2macro = {}

        Agent.__init__(self, name, mapltask, planner, simulator)
        self.statistics = statistics.Statistics(defaults = statistics_defaults)

    def run(self):
        agent.BaseAgent.run(self)
        if global_vars.mapsim_config.learning == "cluster":
            self.restrict_domains(self.task)
            self.clustering()
        elif global_vars.mapsim_config.learning == "learn":
            self.load_macros(global_vars.mapsim_config.macro_file)
            self.restrict_domains(self.task)

            self.macrotask = Task(agent.next_id(), self.mapltask.copy())
            self.planner.register_task(self.macrotask)
            self.restrict_domains(self.macrotask)
            self.learning()
        else:
            #testrun with macros
            self.load_macros(global_vars.mapsim_config.macro_file)
            self.task.mapldomain.actions += self.select_macros()
            Agent.run(self)

    def load_macros(self, filename):
        for m in macros.load_macros(filename, self.task.mapldomain):
            self.add_macro(m)
    
    def add_macro(self, macro):
        log = config.logger("assertions")
        
        for p in macro.pre:
            macro.atom_state[p] = macros.ATOM_DISABLED
        
        superseded = []
        for m in self.macros:
            if macro.less_than(m):
                assert not superseded, "macro database is inconsistent"
                log.debug("A stronger macro than %s already exists: %s", macro.name, m.name)
                m.subsumption_count += 1
                return
            elif m.less_than(macro):
                log.debug("replacing %s with %s", m.name, macro.name)
                superseded.append(m)
        self.macros.append(macro)
        self.name2macro[macro.name] = macro
        for m in superseded:
            macro.subsumption_count += m.subsumption_count
            self.macros.remove(m)
            del self.name2macro[m.name]

            
    def learning(self):
        log.info("%d macros left:", len(self.macros))
        for m in self.macros:
            log.info("%s (%d)", m.name, m.subsumption_count)
            a = m.to_action()
            
        self.learning_run()
        
    def clustering(self):
        self.task.replan()
        self.find_clusters()

        writer = macros.MacroWriter()
        s = writer.write_macros(self.macros)
        f = open(global_vars.mapsim_config.macro_file, "w")
        for line in s:
            f.write(line)
            f.write("\n")
        f.close()

    def execute(self, action, args):
        #no queuing of actions in the learning simulation
        if global_vars.mapsim_config.learning == "learn":
            self.simulator.execute(action, args, self)
        else:
            self.simulator.schedule(action, args, self)
        
    @loggingScope
    @statistics.time_method_for_statistics("clustering_time")
    def find_clusters(self):
        clusters = assertions.make_clusters(self.task.get_plan(), self.task.mapldomain)
        max_index = 0
        for name in self.name2macro:
            if name.startswith("macro") and int(name[5:]) > max_index:
                max_index = int(name[5:])
                
        for i, c in enumerate(clusters):
            i += max_index
            
            log.info("found cluster: %s", map(str, c))
            m = macros.MacroOp.from_cluster(c, self.task.get_plan(), self.task.mapldomain, "macro%d" % i)
            log.info("macro %d:" % i)
            log.info("preconditions:")
            for a in m.pre:
                log.info("  %s", a.pddl_str())
            log.info("effects:")
            for a in m.eff:
                log.info("  %s", a.pddl_str())
                
            self.add_macro(m)

        log.info("%d macros left:", len(self.macros))
        for m in self.macros:
            log.info("%s (%d)", m.name, m.subsumption_count)
            a = m.to_action()
                    
    def restrict_domains(self, task):
        unknown_vars = set()
        indomain = set()
        for svar in task.get_state().iterkeys():
            if svar.modality == mapl.predicates.i_indomain:
                indomain.add(svar)
                unknown_vars.add(svar.nonmodal())

        for svar in indomain:
            del task.get_state()[svar]
                
        for svar in unknown_vars:
            if svar in self.simulator.state:
                idvar = svar.asModality(mapl.predicates.i_indomain, [self.simulator.state[svar]])
                task.get_state()[idvar] = mapl.types.TRUE

        task.mark_changed()

    def select_macros(self):
        total = sum(m.subsumption_count for m in self.macros)
        avg = float(total)/len(self.macros)
        result = []
        for m in self.macros:
            if m.subsumption_count >= avg:
                a = m.to_assertion()
                result.append(a)
        return result

    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        if global_vars.mapsim_config.learning != "learn":
            return Agent.updateTask(self, new_facts, action_status)

        plan = self.macrotask.get_plan()

        if plan is not None and action_status:
            self.last_action.status = action_status
            self.pre_plan.append(self.last_action)
            
        for f in new_facts:
            self.macrotask.get_state().set(f)
            
        self.macrotask.mark_changed()
        
        state = self.macrotask.get_state()
        plan = self.macrotask.get_plan().topological_sort()

        #check for expandable assertions
        for pnode in plan:
            if isinstance(pnode, plans.DummyNode):
                continue

            if pnode.action.replan:
                if self.planner.check_node(pnode, state, replan=True) and "macro" in pnode.action.name:
                    log.info("Assertion (%s %s) is expandable, execution step completed.", pnode.action.name, " ".join(a.name for a in pnode.full_args))
                    self.next_macro_node = pnode
                    return
        
        self.next_macro_node = None
        
        self.macrotask.replan()
        self.execute_plan(self.macrotask)

    def calculate_reward(self, macro, state, baseline, pre_plan, compare_plan):
        return len(pre_plan)-1 + compare_plan.number_of_nodes() - baseline.number_of_nodes()
    
    @loggingScope
    @statistics.time_method_for_statistics("learning_time")
    def learning_run(self):
        log = config.logger("learning")
        self.macrotask.mapldomain.actions += self.select_macros()
        self.task.replan()
        self.macrotask.replan()
        
        self.pre_plan = []
        baseline = self.task.get_plan()
        self.execute_plan(self.macrotask)
        while self.next_macro_node:
            pnode = self.next_macro_node
            plan = self.macrotask.get_plan()
            s = self.macrotask.get_state()
            m = self.name2macro[pnode.action.name]
            mapping = dict((param.name, c) for (param, c) in zip(pnode.action.agents+pnode.action.args+pnode.action.vars, pnode.args))

            satisfied = m.get_satisfied_conditions(self.next_macro_node, s)
            
            read_later = set()
            for pnode in plan.successors(self.next_macro_node):
                print pnode
                read_later |= pnode.preconds
                read_later |= pnode.replanconds
            relevant_effects = self.next_macro_node.effects & read_later

            log.debug("relevant effects:")
            for svar, val in relevant_effects:
                log.debug("%s = %s", str(svar), str(val))
                
            self.task.set_plan(None)
            self.task.set_state(s)
            self.task.mark_changed()
            self.task.replan()
            assert self.task.get_plan()

            log.debug("baseline:")
            log.debug(map(str, baseline.topological_sort()))
            log.debug("executed:")
            log.debug(map(str, self.pre_plan))
            log.debug("new baseline:")
            log.debug(map(str, self.task.get_plan().topological_sort()))
            
            R = self.calculate_reward(self.next_macro_node, s, baseline, self.pre_plan, self.task.get_plan())
            log.info("macro %s has earned a reward of %d", m.name, R)

            #calculate Q for this state
            Q  = 0.0
            for p in m.pre:
                if p not in satisfied:
                    p = p.negate()
                Q += m.Q[p]

            eff_atoms = set()
            m.instantiate(mapping)
            for e in m.eff:
                facts = s.getEffectFacts(e)
                if not (facts & relevant_effects):
                    e = e.negate()
                else:
                    eff_atoms.add(e)
                Q += m.Q[e]
            m.uninstantiate()
            atom_count = len(m.pre)+len(m.eff)

            Q = Q/atom_count
            m.usecount += 1

            log.debug("%s: Q:%.2f, f:%d", m.name, m.q_total(), m.usecount)
            
            #adjust scores
            for p in m.pre:
                if p not in satisfied:
                    p = p.negate()
                old = m.Q[p]
                m.Q[p] +=  alpha*(R - Q)
                m.frequencies[p] += 1
                log.debug("    %s: Q: %.2f -> %.2f", p.pddl_str(), old, m.Q[p])

            for e in m.eff:
                if e not in eff_atoms:
                    e = e.negate()
                old = m.Q[e]
                m.Q[e] +=  alpha*(R - Q)
                m.frequencies[e] += 1
                log.debug("    %s: Q: %.2f -> %.2f", e.pddl_str(), old, m.Q[e])

            log.debug("New total Q: %.2f", m.q_total())
                
            self.macrotask.replan()
            baseline = self.task.get_plan()
            self.pre_plan = []
            self.execute_plan(self.macrotask)

