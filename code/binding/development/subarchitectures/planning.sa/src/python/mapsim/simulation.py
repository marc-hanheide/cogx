
#! /usr/bin/env python
# -*- coding: latin-1 -*-

import threading

import sys, os, os.path
import Queue
import threading
import time
import random
import time
from collections import defaultdict

import config
import constants
from constants import *
import scheduler
import planning
import plans
import utils
import commands
import state
from state import StateVariable
import planning.mapl.sensors
import planning.mapl as mapl
from planning.mapl.conditions import Conjunction, Condition, Atom, Truth
from planning.mapl.types import TypedObject
from planning.mapl.actions import Action
from planning.mapl.effects import Effect
from planning.mapl.predicates import Predicate
from planning.mapsim_planner import MapsimPlanner
import syscall

AGT_DEFAULT = "agents_default"
    

class SubgoalInfo():
    """ helper struct for storing information about subgoals """
    def __init__(self, subgoal_name, subgoal_op, command, requesting_agent, executing_agent):
        self.subgoal_name = subgoal_name
        self.subgoal_op = subgoal_op
        self.command = command
        self.requesting_agent = requesting_agent
        self.executing_agent = executing_agent
        self.state = SubgoalState.SG_CREATED
    def copy(self):
        return SubgoalInfo(*self.as_tuple())
    def as_tuple(self):
        return self.subgoal_name, self.subgoal_op, self.command, self.requesting_agent, self.executing_agent
    def goal_condition(self):
        return self.subgoal_op.precondition


""" This is the common superclass for both simulators and agents
    that enables both to generate planning problems etc."""
class SimulationObject(object):
    def __init__(self):
        self.name = None
        self.time = 0
        # world model
        self.world_state = state.State()
        self.pddl_problem = None
        self._mapl_problem_dirty = True
        self.known_objects = set()
        # planning-related
        self.domain_data = None
        self.planning_memory = plans.PlanningMemory(self)
        self.subgoal_names = []  # temporary subgoals
        # counter for logfile names
        self.counter = 0
        # statistics
        self.stats = utils.Struct(commands=0, commands_executed=0, plans=0, centrally_solvable=False,
                                  solved=False, planning_time=0.0, planning_time_avg=0.0, timeouts=0,
                                  plans_kept=0, passed=0, duration=0, 
                                  report_fn=config.reporter_file_name)
        config.statistics[self] = self.stats
        # currently executable actions/events
        self._possible_actions = None
        self._sensing_actions = None
        self._neg_sensing_actions = None
        self._expandable_assertions = None

    def log(self, *args, **kwargs):
        if "agent" not in kwargs:
            kwargs["agent"] = self
        config.log(*args, **kwargs)

    def _mapl_domain(self):
        return self.domain_data.mapl_domain
    mapl_domain = property(_mapl_domain)

    def mapl_domain_fn(self):
        return self.domain_data.mapl_domain_fn

    def compiled_domains_path(self):
        return self.domain_data.compiled_domains_path

    def pddl_domain_fn(self):
        return self.domain_data.pddl_domain_fn

    def cpddl_domain_fn(self):
        return self.domain_data.cpddl_domain_fn

    def mapl_problem_dirty(self):
        return self._mapl_problem_dirty

    def set_mapl_problem_dirty(self, dirty=True):
        self._mapl_problem_dirty = dirty
        
    def possible_actions(self):
        if self.mapl_problem_dirty():
            self.get_mapl_problem(file=True)
        return self._possible_actions
        
    def expandable_assertions(self):
        if self.mapl_problem_dirty():
            self.get_mapl_problem(file=True)
        return self._expandable_assertions

    def sensing_actions(self):
        if self.mapl_problem_dirty():
            self.get_mapl_problem(file=True)
        return self._sensing_actions, self._neg_sensing_actions
        
    def register_stats(self, stats_keys):
        for key in stats_keys:
            if key not in self.stats.__dict__:
                self.stats.__dict__[key] = None

    def update_time(self, time):
        self.time = time
        self.counter = 0
        self.world_state.set_time(time)

    def same_name(self, agt):
        myname = self.name.lower()
        try:
            res = myname == agt.name.lower()
        except:
            res = myname == str(agt).lower()
        return res

    def unique_string(self):
        cstr = str(self.counter).zfill(2)
        tstr = str(self.time).zfill(2)
        self.counter += 1
        return "%s_t%s_c%s" % (self.name, tstr, cstr)  

    def unique_string_for_setting(self, setting_dict):
        s = "m%(setting)d" % setting_dict
        return s

    def unique_dot_file_name(self, abs_path=True, pm=False):
        pmstr = "pm_" if pm else ""
        tstr = str(self.time).zfill(2)
        fn = "%s_%st%s.dot" % (self.name, pmstr, tstr)
        if abs_path:
            fn = os.path.join(config.current_data_dir, fn)
        return fn

    def find_known_objects(self):
        state = self.world_state
        self.known_objects.update(arg for svar in state for arg in svar.args)
        self.known_objects.update(val for val in state.values())
        tuples = set(t for t in self.known_objects if isinstance(t,tuple))
        for t in tuples:
            self.known_objects.remove(t)
            self.known_objects.update(t)
        self.known_objects.difference_update(BOOLEAN_VALUE_STRINGS)
        return self.known_objects
        
    def determine_command_effect_as_condition(self, command, mapl_domain):
        ground_action_str = command.mapl_action.description
        elmts = ground_action_str.split()
        op_name = elmts[0]
        for action in mapl_domain.actions:
            if action.name == op_name:
                break
        else:
            raise RuntimeError("no matching operator for command '%s'" % ground_action_str)
        params = action.agents + action.parameters + action.variables
        params = [p.name for p in params]
        var_mapping = dict(zip(params, elmts[1:]))
        effs = [eff.literal for eff in action.effects]
        condition = Conjunction(effs)
        cond_str =  condition.mapl_str()
        inst_condition = utils.multiple_replace(cond_str, var_mapping)
        return inst_condition
        
    def create_temporary_subgoal(self, command, mapl_domain):
        count = len(self.subgoal_names)
        sg_name = "sg%d" % count
        self.subgoal_names.append(sg_name)
        sg = self.determine_command_effect_as_condition(command, mapl_domain)
        subgoal_op = planning.create_subgoal_op(config.subgoal_op, sg, sg_name)
        return sg_name, subgoal_op
        
    def read_mapl_task(self, task_fn, path=None):
        if path is None:
            path = config.scenario_path
        task_fn = os.path.join(path, task_fn)
        assert os.path.exists(task_fn), "Task file %s does not exist." % task_fn
        problem = planning.load_mapl_task(task_fn)
        self.adopt_mapl_task(problem)

    def adopt_mapl_task(self, problem):
        problem.planning_agent = self
        self.problem = problem
        self.static_goal = self.problem.goal
        state = self.world_state
        for atom in self.problem.init_conditions:
            svar, val = atom.as_svar_assignment()
            state[svar] = val    

    def build_mapl_problem_str(self, problem):
        obj_def = [o.to_object_declaration() for o in problem.objects]
        facts = set(self.world_state.to_mapl_factlist())
        goal = problem.goal.mapl_str()
        #distinguish between simulation and agents here.
        #simulation should allow all agents to be planning agents
        agent_name = str(self.name)
        planning_agents = set([agent_name])
        if isinstance(self, Simulation):
            planning_agents.update(agt.name for agt in self.agents)
        for name in planning_agents:
            pa_decl = "%s - planning_agent" % name
            if pa_decl not in obj_def:
                obj_def.append(pa_decl)
            pa_activated = "(commited_to_plan %s : true)" % name
            is_pa = "(is_planning_agent %s : true)" % name
            for fact in (pa_activated, is_pa):
                if fact not in facts:
                    facts.add(fact)
        subs = dict(agent_name=agent_name, task_name=problem.task_name, domain_name=problem.domain_name, obj_def=obj_def, facts=facts, goal=goal)
        from string import Template
        templ = Template(GENERIC_TASK_TEMPLATE)
        indent = "  "
        for k in subs:
            lines = subs[k]
            if utils.is_string(lines):
                lines = [lines]
            subs[k] = ["%s%s" % (indent, line) for line in lines]
            subs[k] = "\n".join(subs[k])
        problem = templ.safe_substitute(subs)
        return problem

    def build_pddl_problem_str(self, mapl_prob_str):
        debug = False
        if debug: print "prob:", mapl_prob_str
        mapl_prob = planning.load_mapl_task(mapl_prob_str.splitlines())
        problem_str = mapl_prob.to_pddl_str()
        if debug: print "prob2:", problem_str
        return problem_str

    def get_mapl_problem(self, file, compute_perceptions=True):
        if self.mapl_problem_dirty():
            self.log("building new mapl problem")
            self.problem.init_state = self.world_state
            mapl_problem_str = self.build_mapl_problem_str(self.problem)
            self.pddl_problem = self.build_pddl_problem_str(mapl_problem_str)
            self.log("new mapl problem:\n%s" % self.pddl_problem)
            if file:
                id = self.unique_string()
                for prob_str, ext in ((mapl_problem_str, "mapl"), (self.pddl_problem, "pddl")):
                    fn = "prob_%s.%s" % (id, ext)
                    fn = os.path.join(config.current_data_dir, fn)
                    open(fn, 'w').write(prob_str)
                    self.pddl_problem_file = fn
            if compute_perceptions:
                self.compute_perceptions_and_possible_actions(fn)
            self.set_mapl_problem_dirty(False)
        if file:
            return self.pddl_problem_file
        return self.pddl_problem
    
    def compute_perceptions_and_possible_actions(self, task):
        # should only be called from get_mapl_problem()
        if not self.mapl_problem_dirty():
            return
        self.log("computing perceptions")
        from planning.planner_base.call_planner import full_state_and_enabled_actions
        if task is None:
            task = self.get_mapl_problem(file=True)
        from planning.planner_base.call_planner import full_state_and_enabled_actions
        # TODO: this is called way too often!!!
        facts, actions, assertions = full_state_and_enabled_actions(task, self.cpddl_domain_fn())
        self._expandable_assertions = set(assertions)
        self._possible_actions = set()
        self._sensing_actions = set()
        self._neg_sensing_actions = set()
        for action in actions:
            if action.startswith(PDDL_SENSOR_PREFIX):
                if action.startswith(PDDL_NEG_SENSOR_PREFIX):
                    action = action.replace(PDDL_NEG_SENSOR_PREFIX, PDDL_SENSOR_PREFIX)
                    self._neg_sensing_actions.add(action)
                self._sensing_actions.add(action)
#            else:
            self._possible_actions.add(action)
        assert all(ps in self._sensing_actions for ps in self._neg_sensing_actions)

    def get_sensor_model(self, sname_pddl):
        sname = sname_pddl.replace(PDDL_SENSOR_PREFIX,"")
        sensor = self.mapl_domain.sname2sensor[sname]
        return sensor

    def compute_all_perceptions(self):
        state = self.world_state
        sensors, neg_sensors = self.sensing_actions()
        agt2percepts = defaultdict(dict)  
        for scommand in sensors:
            is_negative = scommand in neg_sensors
            scommand_list = scommand.split()
            sensor = self.get_sensor_model(scommand_list[0])
            svar = sensor.analyze_sensor_command(scommand)
            agt = scommand_list[1].lower()
            percepts = agt2percepts[agt]
            if is_negative:
                params = scommand_list[1:]
                nsvar, nval = sensor.instantiate_neg_condition(params)
                assert nsvar.name == "!"+svar.name
                #self.log("Agent %s NEGATIVELY senses (%s == %s)" % (agt, nsvar, nval))
                percepts[nsvar] = nval
            else:
                if isinstance(self, Simulation):
                    assert svar in state, "State variable '%s' sensed by sensor model is not contained in state!" % svar
                    val = state[svar]
                else:
                    try:
                        val = state[svar]
                    except KeyError:
                        val = None
                percepts[svar] = val
        return agt2percepts

    def execute_plan(self, plan, realize_k_effects=False, simulator=None, vlevel=2):
        assert len(plan) == 1, "This should currently be used for individual commands, not whole plans!"
        ground_action = plan[0]
        assert not ground_action.is_assertion(), "Action %s is an assertion and cannot be executed." % ground_action
        if simulator is None:
            simulator = self
        self.log("attempting execution of %s" % ground_action, vlevel=vlevel)
        world_state = self.world_state
        mapl_prob = self.get_mapl_problem(file=True)
        monitor_result = config.planner.monitor_plan(plan, mapl_prob, self.pddl_domain_fn())
        if not monitor_result.is_executable:
            self.log("Execution unsuccessful.  Monitor output: %s" % syscall.last_call, vlevel=vlevel)
            return False
        k_effect_prefix = DIRECT_K_PREFIX + PREFIX_SEP
        start_state = set(monitor_result.states[0])
        final_state = set(monitor_result.states[-1])
#         new_state = state.State(self.mapl_domain.props2svar_assignments(final_state))
#         expected_changes = world_state.compute_difference(new_state)
        diff = final_state - start_state
        state_update = state.State(self.mapl_domain.props2svar_assignments(diff))
        from_sensing = ground_action.is_sensing_action()  
        active_sense = ground_action.is_sensing_action() # only temporary
        for svar, val in state_update.items():
            if svar.name.startswith(k_effect_prefix):
                if realize_k_effects:
                    # if this is executed by a simulator then don't store knowledge
                    # effects, but really change the beliefs of the corresponding agent
                    prefix, agt_name, new_svar = svar.split_modal_pddl_representation()
                    # TODO: don't use the real value but the one known to the speaker
                    try:
                        real_val = world_state[new_svar]
                    except KeyError:
                        print "Problem during execution of", ground_action, "by", str(self)
                        print "svar:val =", svar, ": ", val
                        print "new_svar", new_svar
                        for sv,val in world_state.iteritems():
                            print "  ", sv, ": ", val
                        raise
                    agent = simulator.get_agent(agt_name)
                    self.log("K-effect in agent %s: %s : %s" % (agt_name, svar, val), vlevel=vlevel)
                    agent.update_kb({new_svar : real_val}, active_sense, from_sensing)
                    agent.set_mapl_problem_dirty()
#                     if ground_action.is_sensing_action():
#                         agent.update_kb({new_svar : real_val}, active_sense=True)
#                     else:
#                         agent.world_state[new_svar] = real_val
#                         agent.set_mapl_problem_dirty()
                else:
                    self.log("K-effect %s : %s" % (svar, val), vlevel=vlevel)
            oldval = world_state.get(svar)
            if oldval != val:
                #self.update_kb({svar : val}, active_sense)
                world_state[svar] = val
                self.log("setting svar %s : %s" % (svar, val), vlevel=vlevel)
        self.set_mapl_problem_dirty()
        return True

    def goal_achieved(self):
        task = self.get_mapl_problem(file=True)
        done = config.planner.goal_achieved(task, self.pddl_domain_fn())
        if done:
            self.log("Agt %s has detected he has reached his goals" % self.name, screen=True)
        return done

    def check_achievability(self, goal_condition):
       return True

    def add_temporary_subgoal(self, sg_info):
        subgoal_name, subgoal_op, command, requesting_agent, executing_agent = sg_info.as_tuple()
        assert self.name.lower() == executing_agent or self == requesting_agent
        self.log("Added new TSG %s: %s" % (subgoal_name, subgoal_op.precondition.mapl_str()), info=INFO_TSG, agent=self)
        self.temp_goals[subgoal_name] = sg_info
        self.problem.objects.append(TypedObject(subgoal_name, "subgoal"))
        self.recompute_goal_formula()

    def recompute_goal_formula(self):
        parts = list(self.static_goal.parts)
        for subgoal_name in self.temp_goals:
            endgoal = Atom("achieved", [subgoal_name])
            parts.append(endgoal)
        new_goal = Conjunction(parts)
        self.problem.goal = new_goal
        self.set_mapl_problem_dirty()
        self.get_mapl_problem(file=True, compute_perceptions=False)
        
    def determine_svar_val(self, command):
        op_name = command.mapl_action.operator
        assert op_name.startswith(TELL_VAL_STR)
        svar_name = op_name[len(TELL_VAL_STR)+1:]
        predicate = self.mapl_domain.get_predicate_by_name(svar_name)
        num_args = len(predicate.arguments)
        # TODO: the following is a hack: we assume that the state var arguments
        # always come last in tell_val. but that's not necessarily true!
        svar_args = command.mapl_action.arguments[-num_args:]
        svar = state.StateVariable([svar_name]+svar_args)
        command.svar = svar
        try:
            val = self.world_state[svar]
            if utils.is_seq_but_not_string(val):
                val = val[0]
        except KeyError:
            val = None
        command.value = val
        return svar, val

    def show_current_state(self):
        self.log("next state...")

    def compute_statistics(self):
        raise NotImplementedError()


##############


class Simulation(SimulationObject):  # not threading.Thread right now
    # some class variables
    import agent, reporter
    agent_class = None  #to be overridden by domain-specific agent class
    reporter_class = reporter.Reporter
    reporter_templates = {}
    def __init__(self):
        if not Simulation.agent_class:
            import agent
            Simulation.agent_class = agent.Agent
        # basic initialisation
        SimulationObject.__init__(self)
        self.name = config.domain_name+"_simulator"
        config.simulation = self
        self.agents = []
        # dicts for accessing agents by name
        self.name2agent = {}
        self.lname2name = {}
        # communication channels simulation <-> agents
        self.sensor_queues = {}
        self.command_queue = None
        # dicts for overriding standard MAPL simulator
        self.command_classes = {}
        #self.sensor_models = [commands.MAPLSensor()]

    def log(self, *args, **kwargs):
        if "agent" not in kwargs:
            kwargs["agent"] = ""
        config.log(*args, **kwargs)

    def setup(self, setting, prob_name):
        config.setting = setting
        if "memory_duration" not in setting:
            setting["memory_duration"] = 0  # no forgetting!
        self.__dict__.update(setting)
        config.setting_str = self.unique_string_for_setting(setting)
        config.setup_log_and_stats()
        config.scenario = config.unique_scenario_name
        self.setup_reporter()

    def setup_reporter_templates(self, reporter):
        reporter.templates = {}

    def setup_reporter(self):
        reporter = self.reporter_class()
        self.setup_reporter_templates(reporter)
        config.reporter = reporter
        config.report_file = open(config.reporter_file_name, "w")

    def connect(self, agent):
        self.agents.append(agent)
        self.name2agent[agent.name] = agent
        self.lname2name[agent.name.lower()] = agent.name

    def get_agent(self, name):
        import agent
        if isinstance(name, agent.Agent):
            return name
        name = self.lname2name[name.lower()]
        return self.name2agent[name]
        
    def agent(self, name):
        asdf
        try:
            return self.name2agent[name]
        except KeyError:
            return self.name2agent[lname2name[name]]

    def update_stats_from_individual_agents(self):
        stat_names = "passed planning_time plans plans_kept timeouts".split()
        for stat_name in stat_names:
            self.stats.__dict__[stat_name] = sum(a.stats.__dict__[stat_name] for a in self.agents)
        
    def store_stats(self):
        stats = self.stats
        self.update_stats_from_individual_agents()
        # add syscall stats
        sys_stats = config.statistics["syscall"]
        for key in sys_stats.__dict__:
            stats.__dict__[key] = sys_stats.__dict__[key]
        # derived stats
        try:
            stats.planning_time_avg = stats.planning_time / stats.plans
        except ZeroDivisionError:
            stats.planning_time_avg = 0.0
        # now write out all stats
        f = config.stats_file
        f.write("#MAPSIM stats:\n")
        def get_stats():
            keys = stats.__dict__
            for key in sorted(keys):
                val = stats.__dict__[key]
                if val is None:
                    try:
                        val = self.__dict__[key]
                    except KeyError:
                        pass
                if val is not None:
                    if isinstance(val,float):
                        val = round(val,2)
                    yield key, val
        lines = ["%s %s" % tup for tup in get_stats()]
        f.write("\n".join(lines))
        f.close()
        self.log("dump stats:")
        self.log("\n".join(lines))
        self.log("end dump stats")

    def advance_time_for_everybody(self, time=None):
        if time is None:
            time = self.time + 1
        #print "time:", self.time
        self.update_time(time)
        config.time = time
        for agt in self.agents:
            agt.update_time(self.time)
        
    def test_plan_existence(self):
        task = self.get_mapl_problem(file=True)
        ff_plan, action_descriptions = config.planner.find_plan(task, self.cpddl_domain_fn())
        plan = plans.extract_plan(ff_plan, action_descriptions)
        self.stats.centrally_solvable = plan is not None
        if self.stats.centrally_solvable:
            self.log("Scenario %s is centrally solvable" % config.scenario, screen=True)
            self.log("MA plan: \n%s" % plan, screen=True)
        return self.stats.centrally_solvable
      
    def send_perceptions(self):
        percepts = self.compute_all_perceptions()
        for agt in self.agents:
            percept = percepts[agt.name.lower()] 
            agt.update_kb(percept)

    def verbalize_request(self, command):
        requesting_agent_name = command.requesting_agent_name()
        requesting_agent = self.get_agent(requesting_agent_name)
        executing_agent_name = command.executing_agent_name()
        executing_agent = self.get_agent(executing_agent_name)
        subgoal_name, subgoal_op = self.create_temporary_subgoal(command, self.mapl_domain)
        mapl_domains = set(simobj.domain_data for simobj in (requesting_agent, executing_agent))
        for domain_data in mapl_domains:
            mapl_domain = domain_data.mapl_domain
            output_name = domain_data.output_fn
            path = domain_data.compiled_domains_path
            mapl_domain.actions.append(subgoal_op)
            mapl_domain.name2actions = None
            mapl_domain.compile_domain_to_pddl_variants(output_name, path)
        sg_info = SubgoalInfo(subgoal_name, subgoal_op, command, requesting_agent, executing_agent)
        # store request for the requesting agent
        requesting_agent.requests_made[command.mapl_action] = subgoal_name
        request_sent = commands.Request(sg_info)
        requesting_agent.add_temporary_subgoal(sg_info)
#         # ...then put request into receiver's message queue
#         request_received = commands.Request(sg_info)
#         executing_agent.messages_received.append(request_received)

    def execute_command(self, command):
        agt = self.get_agent(command.agent)
        config.current_agent = agt
        if isinstance(command, commands.PassCommand):
            return
        self.log("\nExecuting command '%s' by agent %s" % (command, agt), vlevel=2)
        self.stats.commands += 1
        execution_successful = True
        plan = [command.mapl_action]
        if isinstance(command, commands.SpeechAct):
            command.execute(self)
            execution_successful = True  # we just assume here that the physical execution of speech acts succeeds
        elif isinstance(command, (commands.PhysicalAction, commands.SensingAction)):
            execution_successful = self.execute_plan(plan, realize_k_effects=True)
        agt.react_to_execution(command, execution_successful)
        config.reporter.report_on(command, execution_successful)
        if execution_successful:
            self.stats.commands_executed += 1
        if config.keypress_after_each_action:
            utils.wait_for_keypress()
        config.current_agent = None

    def load_scenario(self):
        config_keys = "num_agents commands plans " \
        "solvable solved time planning_time timeouts plans_kept".split()
        self.register_stats(config_keys)
        fn = os.path.join(config.scenario_path, SCENARIO_CONFIG_FN)
        props = utils.read_property_file(fn)
        self.read_mapl_task(props["world"])
        self.num_agents = props["agents"]
        agent_names = [n for n in props if props[n]==""]
        assert len(agent_names) == self.num_agents, \
               "%s is inconsistent: %d agents defined instead of the promised %d ones" % \
               (fn, len(agent_names), self.num_agents)
        for name in agent_names:
            agt = self.agent_class(self, name=name)
            fn = "%s.mapl" % name
            agt.read_mapl_task(fn)

    def show_initial_setting_and_problem(self):
        """can be overridden to show the global and individual planning problem before the start of the simulation."""
        agts_sorted = sorted(self.agents, key=str)
        for agt in agts_sorted:
            goal_str = " ".join(agt.original_goal.pddl_str().split())
            self.log("%s's goal: %s" % (agt, goal_str), vlevel=REPORTER)
        
    def once_per_round_stuff(self):
        self.advance_time_for_everybody()
        self.send_perceptions()
        if config.clear_screen_each_round:
            utils.clear_screen()
        self.log("\nTIME %d:     (%s)" % (self.time, config.scenario), screen=True, vlevel=4)
        self.show_current_state()
        if config.keypress_after_each_round:
            utils.wait_for_keypress()

    def setup_planning_domains(self, dom_name):
        config.compiled_domains = {}
        simulator_domain_fn = os.path.join(config.domains_path, dom_name, "mapl_files/domain.mapl")
        agents_default_domain_fn = os.path.join(config.domains_path, dom_name, "mapl_files/domain_agents.mapl")
        domain_names = {self:simulator_domain_fn}
        domain_output_names = {self:"domain"}
        for agt in self.agents:
            agt_domain_fn = "domain_%s.mapl" % agt.name
            fn = os.path.join(config.scenario_path, agt_domain_fn)
            domain_names[agt] = fn
            domain_output_names[agt] = "domain_" + agt.name

        config.domain_data = {}
        for simobj in [self] + list(self.agents):
            fn = domain_names[simobj]
            output = domain_output_names[simobj]

            if os.path.exists(domain_names[simobj]):
                fn = domain_names[simobj]
            elif os.path.exists(agents_default_domain_fn):
                # fallback 1: use default agent domain
                fn = agents_default_domain_fn
            else:
                # fallback 2: use simulator domain
                fn = config.domain_data[self].mapl_domain_fn

            dd = mapl.DomainData()
            dd.mapl_domain_fn = fn
            dd.mapl_domain = mapl.load_mapl_domain(fn)
            dd.planning_agent = simobj
            dd.mapl_domain.planning_agent = simobj
            dd.output_fn = output
            dd.compiled_domains_path = config.current_data_dir

            # create a pseudo-action that fulfills the goal and add it to the domain
            goal_action, goal_conj = self.create_goal_pseudo_action(simobj.problem.goal, simobj.name)
            simobj.original_goal = simobj.problem.goal
            simobj.problem.goal = goal_conj

            pred = Predicate(GOAL_REACHED_PREDICATE, [TypedObject("?a", "planning_agent")], None)
            dd.mapl_domain.predicates[GOAL_REACHED_PREDICATE] = pred
            dd.mapl_domain.actions += [goal_action]
            dd.mapl_domain.name2actions = None

            dd.pddl_domain_fn, dd.cpddl_domain_fn = dd.mapl_domain.compile_domain_to_pddl_variants(output, dd.compiled_domains_path)

            config.domain_data[simobj] = dd
            simobj.domain_data = dd
            
            simobj.problem.mapl_domain = dd.mapl_domain

    def create_goal_pseudo_action(self, goal, name):
        """Returns an action to the domain that results in (GOAL_REACHED_PREDICATE agent) being true,
        and a conjunction that contains this literal which can then be used as the goal"""
        agent = TypedObject("?a", "planning_agent")
        action_name = GOAL_PSEUDO_ACTION+PREFIX_SEP + name.lower()
        parameters = []
        eff_literal = Atom(GOAL_REACHED_PREDICATE, [name.lower()])
        #print eff_literal
        effect = Effect([], Truth(), eff_literal)
        goal_action = Action(action_name, parameters, goal, [effect], [agent])
        
        return goal_action, Conjunction([eff_literal])
                
    def simulation_loop(self):
        """ the main simulation process is executed from here """
        scheduler = self.scheduler
        self.once_per_round_stuff()
        while True:
            command = scheduler.ask_for_next_command()
            if isinstance(command, commands.StopSimulationCommand):
                break
            execution_result = self.execute_command(command)
            scheduler.execution_result(command, execution_result)
            if not isinstance(command, commands.PassCommand):
                self.once_per_round_stuff()
        assert isinstance(command, commands.StopSimulationCommand)
        msg_verbosity = {True: 3, False: 1}[command.success]
        self.log("MAPSIM will stop now: %s" % command.reason, screen=True, vlevel=msg_verbosity)
        self.stats.solved = command.success

    def output_start_msg(self):
        num_agents = len(self.agents)
        agt_names = " and ".join(sorted(agt.name for agt in self.agents))
        agt_names = agt_names.replace(" and", ",", num_agents-2)
        msg = "is 1 agent" if num_agents == 1 else "are %d agents" % num_agents
        msg = "MAPSIM run starts. There %s: %s" % (msg, agt_names)
        self.log("MAPSIM is starting a new run for scenario '%s'." % config.scenario, screen=True)
        config.reporter.log_and_report(msg)

    def run(self):
        self.load_scenario()
        self.setup_planning_domains(config.domain_name)
        config.planner = MapsimPlanner()
        self.output_start_msg()
        self.show_initial_setting_and_problem()
        self.show_current_state()
        start_time = time.time()
        self.scheduler = scheduler.Scheduler(self)
        plan_exists = True #self.test_plan_existence()
        planex_time = time.time()
        if not plan_exists:
            self.log("MAPSIM terminates run. Scenario '%s' proved unsolvable." % config.scenario, screen=True)
            return
        # now do the real work:
        self.simulation_loop()
        # that wasn't so hard, was it?
        duration = time.time() - start_time
        self.stats.duration = duration
        success = "successfully" if self.stats.solved else "unsuccessfully"
        config.reporter.log_and_report("MAPSIM terminates %s." % success)
        self.log("Runtime for scenario %s: %.1f secs.\n\n" % (config.scenario, round(duration,1)), vlevel=1)
        self.log("Writing statistics of this run to %s." % config.stats_fn)
        self.store_stats()
        return self.stats

def compiled_domain_is_dated(mapl_domain_fn):
    pddl_domain_file = mapl_domain_fn.replace(".mapl", ".pddl")
    continual_pddl_domain_file = mapl_domain_fn.replace(".mapl", ".cpddl")
    compiled_files = (pddl_domain_file, continual_pddl_domain_file)
    return any(utils.is_dated(compiled_file, mapl_domain_fn) for compiled_file in compiled_files)
           
