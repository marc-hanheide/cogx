# -*- coding: utf-8 -*-
import math, random, itertools

from standalone import pddl
from standalone.pddl import state, mapl
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
log = config.logger("learning")

statistics_defaults = dict(
    learning_time=0,
    clustering_time=0
    )
statistics_defaults.update(agent.statistics_defaults)


class LearningAgent(Agent):
    def __init__(self, name, mapltask, planner, simulator):
        self.macros = []
        self.name2macro = {}

        Agent.__init__(self, name, mapltask, planner, simulator)
        self.statistics = statistics.Statistics(defaults = statistics_defaults)

        if global_vars.mapsim_config.learning_mode != "cluster":
            self.load_macros(global_vars.mapsim_config.macro_file)
            
        
    def run(self):
        agent.BaseAgent.run(self)
        if global_vars.mapsim_config.learning_mode == "cluster":
            self.restrict_domains(self.task)
            self.clustering()
        elif global_vars.mapsim_config.learning_mode == "learn":
            self.restrict_domains(self.task)
            newtask = self.mapltask.copy()
            newtask.domain = self.mapltask.domain.copy()
            self.macrotask = Task(agent.next_id(), newtask)
            self.macrotask.set_state(self.task.get_state().copy())
            self.planner.register_task(self.macrotask)
            self.restrict_domains(self.macrotask)
            self.learning()
            for m in self.macros:
                self.adjust_macro(m)
        else:
            #testrun with macros
            self.task.mapldomain.actions += self.select_macros_final()
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
        log.info("%d macros:", len(self.macros))
        for m in self.macros:
            log.info("%s (%d)", m.name, m.subsumption_count)
            a = m.to_action()
            
        self.learning_run()
        self.write_macros(global_vars.mapsim_config.macro_file+".out")
        
        
    def clustering(self):
        self.task.replan()
        self.find_clusters()
        self.write_macros(global_vars.mapsim_config.macro_file)

    def write_macros(self, filename):
        writer = macros.MacroWriter()
        s = writer.write_macros(self.macros)
        f = open(filename, "w")
        for line in s:
            f.write(line)
            f.write("\n")
        f.close()

    def execute(self, action, args):
        #no queuing of actions in the learning simulation
        if global_vars.mapsim_config.learning_mode == "learn":
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
            if svar.modality == mapl.i_indomain:
                indomain.add(svar)
                unknown_vars.add(svar.nonmodal())

        for svar in indomain:
            del task.get_state()[svar]
                
        for svar in unknown_vars:
            if svar in self.simulator.state:
                idvar = svar.as_modality(mapl.i_indomain, [self.simulator.state[svar]])
                task.get_state()[idvar] = pddl.TRUE

        task.mark_changed()

    def select_macros_for_learning(self):
        total = sum(m.subsumption_count for m in self.macros)
        avg = float(total)/len(self.macros)
        result = []
        for m in self.macros:
            if m.subsumption_count >= avg*2:
                a = m.to_assertion()
                result.append(a)
        return result

    def select_macros_final(self):
        total = sum(m.usecount for m in self.macros)
        avg = max(1, float(total)/len(self.macros))
        
        result = []
        for m in self.macros:
            if m.usecount >= avg:
                log.info("Use macro: %s" % m.name)
                a = m.to_assertion()
                result.append(a)
            else:
                log.info("Don't use macro: %s" % m.name)
                
        return result
    
    @loggingScope
    def updateTask(self, new_facts, action_status=None):
        if global_vars.mapsim_config.learning_mode != "learn":
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

        self.expandable_macros = []
        #check for expandable assertions
        for pnode in plan:
            if isinstance(pnode, plans.DummyNode):
                continue

            if pnode.action.replan:
                if self.planner.check_node(pnode, state, replan=True) and "macro" in pnode.action.name:
                    log.info("Assertion (%s %s) is expandable, execution step completed.", pnode.action.name, " ".join(a.name for a in pnode.full_args))
                    self.expandable_macros.append(pnode)

        if self.expandable_macros:
            return
        
        self.macrotask.replan()
        self.execute_plan(self.macrotask)

    def calculate_reward(self, macro, relevant_effects, state, baseline, pre_plan, compare_plan):
        settings = global_vars.mapsim_config.learning
        #check which effects could be realised in the compare_plan:
        fulfilled_svars = set()
        for pnode in compare_plan.nodes_iter():
            fulfilled_svars |= pnode.effects & relevant_effects

        sensed_svars = set()
        for svar, val in itertools.chain(macro.original_replan):
            if svar.modality == mapl.knowledge:
                sensed_svars.add(svar.nonmodal())

        useful_svars_full = set()
        for pnode in compare_plan.nodes_iter():
            useful_svars_full |= set(f.svar.nonmodal() for f in pnode.preconds)

        useful_ratio_full = float(len(useful_svars_full & sensed_svars)) / len(sensed_svars)
        overhead = len(pre_plan) + compare_plan.number_of_nodes() - baseline.number_of_nodes()
        if not relevant_effects:
            success_ratio = 0
        else:
            success_ratio = float(len(fulfilled_svars)) / len(relevant_effects)
        
        log.info("I = %.2f, S = %.2f, O = %.2f", useful_ratio_full, success_ratio, overhead)

        return settings.success_weight*(success_ratio - 1) + settings.information_weight*(useful_ratio_full - 1) - settings.overhead_weight*overhead

    def find_expanded_macro(self, macro, relevant_effects, plan, compare_plan):
        frontier = []
        for pnode in compare_plan.nodes_iter():
            if pnode.effects & relevant_effects:
                frontier.append(pnode)
                
        if not frontier:
            return None

        expanded = set(frontier)
        while frontier:
            pnode = frontier.pop()
            if pnode.original_preconds & macro.original_preconds:
                continue
            for pred in compare_plan.predecessors(pnode):
                if pred not in expanded and not isinstance(pred.action, plans.DummyAction):
                    expanded.add(pred)
                    frontier.append(pred)
        return expanded
                
    
    @loggingScope
    @statistics.time_method_for_statistics("learning_time")
    def learning_run(self):
        settings = global_vars.mapsim_config.learning

        self.macrotask.mapldomain.actions += self.select_macros_for_learning()
        self.task.replan()
        self.macrotask.replan()

        turn = 0
        
        self.pre_plan = []
        baseline = self.task.get_plan()
        self.execute_plan(self.macrotask)
        while self.expandable_macros:
            plan = self.macrotask.get_plan()
            s = self.macrotask.get_state()

            self.task.set_plan(None)
            self.task.set_state(s)
            self.task.mark_changed()
            self.task.replan()
            assert self.task.get_plan()

            if global_vars.mapsim_config.write_pdffiles:
                G = plan.to_dot()
                G.layout(prog='dot')
                G.draw("macroplan%d.pdf" % turn)

                G = baseline.to_dot()
                G.layout(prog='dot')
                G.draw("baseline%d.pdf" % turn)

                G = self.task.get_plan().to_dot()
                G.layout(prog='dot')
                G.draw("nextbaseline%d.pdf" % turn)

            log.debug("baseline:")
            log.debug(map(str, baseline.topological_sort()))
            log.debug("executed:")
            log.debug(map(str, self.pre_plan))
            log.debug("new baseline:")
            log.debug(map(str, self.task.get_plan().topological_sort()))
            
            for pnode in self.expandable_macros:
                m = self.name2macro[pnode.action.name]
                m.parent = self.task.mapltask
                mapping = dict((param.name, c) for (param, c) in zip(pnode.action.args, pnode.args))

                satisfied = m.get_satisfied_conditions(pnode, s)

                read_later = set()
                for snode in plan.successors(pnode):
                    read_later |= snode.preconds
                    read_later |= snode.replanconds
                relevant_effects = pnode.effects & read_later

                log.debug("relevant effects:")
                for svar, val in relevant_effects:
                    log.debug("%s = %s", str(svar), str(val))

                expanded = self.find_expanded_macro(pnode, relevant_effects, plan, self.task.get_plan())
                if expanded:
                    log.debug("expanded macro is: %s", map(str, sorted(expanded, key=lambda x: x.time)))

                R = self.calculate_reward(pnode, relevant_effects, s, baseline, self.pre_plan, self.task.get_plan())
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
                    if m.atom_state[e] == macros.ATOM_DISABLED:
                        e = e.negate()
                    else:
                        facts = s.get_effect_facts(e)
                        if not (facts & relevant_effects):
                            e = e.negate()
                        else:
                            eff_atoms.add(e)
                    Q += m.Q[e]
                m.uninstantiate()
                atom_count = len(m.pre)+len(m.eff)

                Q = Q/atom_count
                m.usecount += 1

                log.info("%s: Q:%.2f, total Q: %.2f, f:%d", m.name, Q, m.q_total(), m.usecount)
                alpha = 1 - math.pow(1 - settings.alpha, m.alpha_boost)
                if m.alpha_boost > 1:
                    log.info("Alpha boost: %d, effective α: %.2f", m.alpha_boost, alpha)
                    m.alpha_boost -= 1

                #adjust scores
                for p in m.pre:
                    if p not in satisfied:
                        p = p.negate()
                    old = m.Q[p]
                    m.Q[p] +=  alpha*(R - Q)
                    m.frequencies[p] += 1
                    log.info("    %s: Q: %.2f -> %.2f", p.pddl_str(), old, m.Q[p])

                for e in m.eff:
                    if e not in eff_atoms:
                        e = e.negate()
                    old = m.Q[e]
                    m.Q[e] +=  alpha*(R - Q)
                    m.frequencies[e] += 1
                    log.info("    %s: Q: %.2f -> %.2f", e.pddl_str(), old, m.Q[e])
                    
                if expanded:
                    old = m.costs
                    costs = len(expanded)
                    m.costs += alpha*(costs - m.costs)
                    log.info("    costs: %.2f -> %.2f", old, m.costs)

                log.info("New total Q: %.2f, expected Q: %.2f", m.q_total(), m.q_expected())

            turn += 1
            self.macrotask.replan()
            baseline = self.task.get_plan()
            self.pre_plan = []
            self.execute_plan(self.macrotask)

    def adjust_macro(self, m):
        settings = global_vars.mapsim_config.learning
        
        if m.usecount == 0:
            return
        
        best_atom = None
        max_gain = 0
        log.info("Adjusting %s:", m.name)
        for atom in itertools.chain(m.pre, m.eff):
            dQ = (m.Q[atom] - m.Q[atom.negate()]) / (len(m.pre)+len(m.eff))
            mark = " "
            if m.atom_state[atom] == macros.ATOM_ENABLED:
                dQ = -dQ
                mark = "x"

            if m.frequencies[atom] == 0 or m.frequencies[atom] == m.usecount:
                log.info("   %s %s with dQ=%.2f, f_rel=%.2f skipped", mark, atom.pddl_str(), dQ, float(m.frequencies[atom])/m.usecount)
                continue
            threshold = settings.error_threshold * (1/math.sqrt(m.frequencies[atom]) +  1/math.sqrt(m.frequencies[atom.negate()]))
            
            log.info("   %s %s with dQ=%.2f, threshold=%.2f, f_rel=%.2f", mark, atom.pddl_str(), dQ, threshold, float(m.frequencies[atom])/m.usecount)
            if dQ > threshold and dQ > max_gain:
                best_atom = atom
                max_gain = dQ

        if best_atom:
            if m.atom_state[best_atom] == macros.ATOM_ENABLED:
                m.atom_state[best_atom] = macros.ATOM_DISABLED
                action = "Disabled"
            else:
                m.atom_state[best_atom] = macros.ATOM_ENABLED
                action = "Enabled"

            log.info("%s atom %s with dQ=%.2f", action, atom.pddl_str(), max_gain)
            m.alpha_boost = settings.alpha_boost
            
            for a,f in m.frequencies.iteritems():
                m.frequencies[a] = f * settings.usecount_reduction
            m.usecount *= settings.usecount_reduction

        else:
            log.info("No changes")
            
