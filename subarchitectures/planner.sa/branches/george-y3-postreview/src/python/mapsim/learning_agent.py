# -*- coding: utf-8 -*-
import os, re, glob, math, random, itertools

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

        global_vars.mapsim_config.add_assertions = False
        self.macro_file_index = -1
        
        #mapltask.optimization = "minimize"
        #mapltask.opt_func = pddl.Term(pddl.builtin.total_cost, [])
        newtask = mapltask.copy()
        newtask.domain = mapltask.domain.copy()
        self.macrotask = Task(agent.next_id(), newtask)
        
        Agent.__init__(self, name, mapltask, planner, simulator)
        self.statistics = statistics.Statistics(defaults = statistics_defaults)

        have_macros = False
        try:
            filename = self.get_macro_input_file()
            log.info("Using macro db: %s", filename)
            print filename
            for m in macros.load_macros(filename, self.task.mapldomain):
                have_macros = True
                if global_vars.mapsim_config.learning_mode in ("learn", "cluster"):
                    self.add_macro(m)
                elif global_vars.mapsim_config.learning_mode == "test":
                    self.add_macro(m, enabled_only=True)
        except (pddl.parser.ParseError, IOError):
            self.macro_file_index = -1

        if global_vars.mapsim_config.learning_mode == "learn":
            self.task.add_assertions()
        if global_vars.mapsim_config.learning_mode == "cluster":
            self.task.add_assertions()
            if not have_macros:
                for a in self.task.mapltask.actions:
                    if a.name.startswith("assertion_"):
                        m = macros.MacroOp.from_action(a, self.task.mapltask)
                        m.name = "macro_" + m.name
                        self.add_macro(m)
        
        
    def run(self):
        agent.BaseAgent.run(self)
        if global_vars.mapsim_config.learning_mode == "cluster":
            self.restrict_domains(self.task)
            self.clustering()
        elif global_vars.mapsim_config.learning_mode == "learn":
            self.restrict_domains(self.task)
            self.macrotask.set_state(self.task.get_state().copy())
            self.planner.register_task(self.macrotask)
            self.restrict_domains(self.macrotask)
            self.learning()
            for m in self.macros:
                self.adjust_macro(m)
        else:
            #testrun with macros
            self.task.mapldomain.actions = [a for a in self.task.mapldomain.actions if a.name not in self.name2macro]
            self.task.mapldomain.actions += self.select_macros_final()
            Agent.run(self)

    def get_macro_input_file(self):
        version = global_vars.mapsim_config.macro_version
        basename = global_vars.mapsim_config.macro_filename
        basepath = global_vars.mapsim_config.macro_path

        if basename is None:
            basename = self.mapltask.domain.name.lower()

        basefilename = os.path.join(basepath, basename + ".mapl")
        
        if version is not None:
            self.macro_file_index = version
            if global_vars.mapsim_config.macro_version == 0:
                fname = basefilename
            else:
                fname = basefilename + ".%d" % version
            if os.path.exists(fname):
                return fname
        
        latest = -1
        
        regexp = re.compile("\.([0-9]*)$")
        for fname in glob.glob(basefilename + ".[0-9]*"):
            index = regexp.search(fname).group(1).lower()
            if index and int(index) > latest:
                latest = int(index)

        if latest == -1:
            self.macro_file_index = 0
            return basefilename
        
        self.macro_file_index = latest
        return basefilename + ".%d" % latest

    def get_macro_output_file(self):
        version = self.macro_file_index
        basename = global_vars.mapsim_config.macro_filename
        basepath = global_vars.mapsim_config.macro_path

        if basename is None:
            basename = self.mapltask.domain.name.lower()
        basefilename = os.path.join(basepath, basename + ".mapl")

        if version == -1:
            return basefilename
        return basefilename + ".%d" % (version+1)
    
        
    def add_macro(self, macro, enabled_only=False):
        log = config.logger("assertions")

        astate = macro.atom_state.copy()
        for p in macro.pre:
            macro.atom_state[p] = macros.ATOM_DISABLED
        
        superseded = []
        for m in self.macros:
            if macro.less_than(m, enabled_only):
                assert not superseded, "macro database is inconsistent"
                log.debug("A stronger macro than %s already exists: %s", macro.name, m.name)
                m.subsumption_count += 1
                return
            elif m.less_than(macro, enabled_only):
                log.debug("replacing %s with %s", m.name, macro.name)
                superseded.append(m)

        macro.atom_state = astate
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
        self.write_macros(self.get_macro_output_file())
        
        
    def clustering(self):
        self.task.replan()
        if global_vars.mapsim_config.write_pdffiles:
            G = self.task.get_plan().to_dot()
            G.layout(prog='dot')
            G.draw("plan%d.pdf" % self.simulator.run_index)

        self.find_clusters()
        self.write_macros(self.get_macro_output_file())

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
            if "assertion" in name:
                continue
            if name.startswith("macro") and int(name[5:]) > max_index:
                max_index = int(name[5:])
        
        for i, c in enumerate(clusters):
            i += max_index + 1
            
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
        if not self.macros:
            return []
        
        total = sum(m.subsumption_count for m in self.macros)
        avg = float(total)/len(self.macros)
        result = []
        for m in self.macros:
            if "assertion" in m.name or \
                    float(m.subsumption_count) / total >= global_vars.mapsim_config.learning.min_subsumption_ratio:
                oldstate = m.atom_state.copy()
                for a in m.pre:
                    if m.atom_state[a] == macros.ATOM_DISABLED and random.random() > 0.7:
                        m.atom_state[a] = macros.ATOM_ENABLED
                
                a = m.to_assertion()
                m.atom_state = oldstate
                result.append(a)
                log.info("Learning macro %s", m.name)
        return result

    def select_macros_final(self):
        if not self.macros:
            return []

        settings = global_vars.mapsim_config.learning
        
        total = sum(m.usecount for m in self.macros if "assertion" not in m.name)
        usecount_threshold = int(settings.min_desired_macro_usage * total)
        current_usecount = 0
        current_macrocount = 0

        whitelist = set()
        if global_vars.mapsim_config.enable_macros:
            whitelist = set(global_vars.mapsim_config.enable_macros.split(","))
        
        result = []
        for m in sorted(self.macros, key=lambda m: m.usecount):
            for eff in m.eff:
                if m.frequencies[eff] < 0.05:
                    m.atom_state[eff] == macros.ATOM_DISABLED
                    
            if m.name in whitelist:
                log.info("Use macro in whitelist: %s" % m.name)
                a = m.to_assertion()
                result.append(a)
                continue
            
            if m.usecount <= 0:
                log.info("Don't use macro: %s, never been used" % m.name)
                continue
            
            if m.q_expected() < settings.q_min:
                log.info("Don't use macro: %s, Q(%.2f) < %.2f" % (m.name, m.q_expected(), settings.q_min))
                continue
            
            if "assertion" in m.name:
                log.info("Use macro from simple assertion: %s" % m.name)
                a = m.to_assertion()
                result.append(a)
                continue

            if current_macrocount >= settings.max_macro_count:
                log.info("Don't use macro: %s, Maximum number is reached" % m.name)
                continue
                
            if current_usecount >= usecount_threshold:
                log.info("Don't use macro: %s, Usecount ratio of %.2f = %d is reached" % (m.name, settings.min_desired_macro, usecount_threshold))
                continue
                 
            log.info("Use macro: %s" % m.name)
            a = m.to_assertion()
            result.append(a)

        for a in itertools.chain(self.task.mapldomain.actions, self.task.mapldomain.sensors):
            if assertions.to_assertion(a, self.task.mapldomain):
                m = macros.MacroOp.from_action(a, self.task.mapldomain)
                m.name = "assertion_" + m.name
                self.name2macro[m.name] = m
                
                for a in m.pre:
                    m.atom_state[a] = macros.ATOM_ENABLED

                if not any(m.less_than(self.name2macro[a.name], enabled_only=True) for a in result):
                    log.info("Add simple assertion: %s" % m.name)
                    result.append(m.to_assertion())
                    
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
        start_frontier = set()
        end_frontier = []
        for pnode in compare_plan.nodes_iter():
            if pnode.effects & relevant_effects:
                end_frontier.append(pnode)
                
        if not end_frontier:
            return None

        expanded = set(end_frontier)
        while end_frontier:
            pnode = end_frontier.pop()
            if pnode.original_preconds & macro.original_preconds:
                start_frontier |= compare_plan.pred_closure(pnode, link_type='depends')
                continue
            if pnode in start_frontier:
                continue
            for pred in compare_plan.predecessors(pnode):
                if pred not in expanded and not isinstance(pred.action, plans.DummyAction) and \
                        any(e['type'] == 'depends' for e in compare_plan[pred][pnode].itervalues()):
                    expanded.add(pred)
                    end_frontier.append(pred)
        return expanded
                
    
    @loggingScope
    @statistics.time_method_for_statistics("learning_time")
    def learning_run(self):
        settings = global_vars.mapsim_config.learning

        self.macrotask.mapldomain.actions = [a for a in self.macrotask.mapldomain.actions if a.name not in self.name2macro]
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
                log.debug("expanded macro: %s", str(pnode))
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

        threshold = settings.error_threshold * 1/math.sqrt(m.usecount)
        if m.q_expected() > settings.q_min + threshold:
            log.info("No need to improve %s:", m.name)
            log.info("Expected Q: %.2f", m.q_expected())
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
                
            assert m.frequencies[atom] + m.frequencies[atom.negate()] > m.usecount - 1e-6, "%s: %d + %d != %d" % (atom.pddl_str(), m.frequencies[atom], m.frequencies[atom.negate()], m.usecount)
            
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

            log.info("%s atom %s with dQ=%.2f", action, best_atom.pddl_str(), max_gain)
            m.alpha_boost = settings.alpha_boost
            
            for a,f in m.frequencies.iteritems():
                m.frequencies[a] = f * settings.usecount_reduction
            m.usecount *= settings.usecount_reduction

        else:
            log.info("No changes")
            
        log.info("Expected Q: %.2f", m.q_expected())
            
