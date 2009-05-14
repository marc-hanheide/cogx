from collections import defaultdict, deque
from constants import *
from simulation import SimulationObject, SubgoalInfo
from utils import it2str, bit_is_set
import Queue
import commands
import config
import idl_classes  
import infostate
import os.path
import planning
import plans
import random
import state
import syscall
import threading
import time

class Agent(threading.Thread, SimulationObject):
    def __init__(self, simulator=None, name=None):
        threading.Thread.__init__(self)
        SimulationObject.__init__(self)
        if not name:
            self.id = len(simulator.agents)
            name = AGENT_NAME_TMPL % self.id
        else:
            self.id = name
        self.name = name
        self.sensor_queue = Queue.Queue(1)
        self.messages_received = deque()
        self.simulator = simulator
        memory_duration = 0
        self.world_state = state.State(memory_span=memory_duration)
        self.last_state = state.State(memory_span=memory_duration)
        self.requests_made = {} # maps requested action -> subgoal name
        self.temp_goals = dict()  # sg_name --> (sg_name, request, requesting_agent, executing_agent)
        self.static_goal = None
        self.planning_memory = plans.PlanningMemory(self)
        self.current_plan = None
        # remember some history
        self.information_state = infostate.InformationState(self)
        self._next_commands = []  # accumulate possible next actions
        # ...and off we go
        if simulator:
            simulator.connect(self)

    def __str__(self):
        return self.name

    def get_current_plan(self):
        return self.planning_memory.current_plan()

    def novel_percepts(self, percept, world_state):
        def np_gen():
            for (k,v) in percept.items():
                if k.is_negative():   # negative sensing
                    nk = k.invert()
                    oldv = world_state.get(nk)
                    if oldv == v or isinstance(oldv,tuple) and oldv[0] == v:
                        # v was the value of k previously, but now is negatively sensed
                        yield (k,v)
                    elif world_state.get(nk.in_domain_proposition(v)) == TRUE_TUP:
                        # v was a possible value vor k, but now is negatively sensed
                        yield (k,v)
                elif world_state.get(k) != v:
                    # k hat a differenent value previously
                    yield (k,v)
        return dict(np_gen())
    
    def relevant_percepts(self, percept, world_state):
        """parameters: percept: set(StateVariable)
                       state:   State
           returns dict(StateVariable => value(?))"""
        def rp_gen(relevant):
            for svar in relevant:
                if svar.is_own_knowledge(self.name):
                    yield svar.base_of_knowledge_proposition(self.name)
                    yield svar.base_of_knowledge_proposition(self.name).invert()
                elif svar.is_in_domain():
                    yield svar.base_of_in_domain_proposition()
                    yield svar.base_of_in_domain_proposition().invert()
                else:
                    yield svar
                    yield svar.invert()
                    
        if self.get_current_plan().is_empty():
            return dict()
        
        novel_percepts = self.novel_percepts(percept, world_state)
        relevant_vars = set(rp_gen(plans.get_relevant_svars(self.get_current_plan())))

        #print "rel:", [ str(var) for var in relevant_vars]
        #print "new:", [ str(var) for var,_ in novel_percepts.iteritems()]
        
        return dict([ (k,v) for k,v in novel_percepts.iteritems() if k in relevant_vars ])

    def expected_perceptions(self):
        percepts = self.compute_all_perceptions()
        my_percepts = percepts[self.name.lower()]
        # determine those perceptions that are new with respect to the last state estimation
        novel_percepts = self.novel_percepts(my_percepts, self.last_state)
        return novel_percepts

    def update_kb(self, percept, active_sense=False, from_sensor=True):
        if config.only_do_relevant_sensing: # and not self.get_current_plan().is_empty():
            novel_percepts = self.relevant_percepts(percept, self.world_state)
        else:
            novel_percepts = self.novel_percepts(percept, self.world_state)

        #force this update if this is an active sense
        if not novel_percepts and active_sense:
            novel_percepts = percept

        #print [ str(var) for var,_ in novel_percepts.iteritems()]
        
        if True:  # for testing purposes
            expected_percepts = self.expected_perceptions()
            unexpected_percepts = dict((k,v) for (k,v) in novel_percepts.items() if v != expected_percepts.get(k))
            falsely_expected_percepts = dict((k,v) for (k,v) in expected_percepts.items() if v != percept.get(k))
            for k,v in unexpected_percepts.items():
                self.log("UE: did not expect %s" % (state.str_assignment(k,v)))
            for k,v in falsely_expected_percepts.items():
                self.log("FE: expected %s, but got %s " % (state.str_assignment(k,v), percept.get(k)))
            
        wstate = self.world_state
        for (k,v) in novel_percepts.items():
            self.set_mapl_problem_dirty()
            if k.name[0] == "!":
                nname = k.name[1:]
                k = state.StateVariable((nname,)+k.args)
                oldv = wstate.get(k)
                if oldv == v or isinstance(oldv,tuple) and oldv[0] == v:
                    self.log("negative sensing: remove old val. now %s != %s" % (k,v))
                    del wstate[k]
                id_svar = k.in_domain_proposition(v)
                if id_svar in wstate:
                    self.log("negative sensing: remove from domain. now: %s != %s" % (k,v))
                    #del wstate[id_svar]
                    wstate[id_svar] = FALSE_TUP
                if from_sensor and bit_is_set(config.sensing_mode, REPORT_NEG_SENSING):
                    config.reporter.report_sensing(self, k, v, negative=True, active_sense=active_sense)
            elif wstate.get(k) != v:
                self.log("sensing (%s = %s)" % (k,v))
                wstate[k] = v
                id_svar = k.in_domain_proposition(v)
                removals = set()
                for idk in wstate:
                    if idk.name == id_svar.name and idk.args[:-1] == id_svar.args[:-1]:
                        removals.add(idk)
                for idk in removals:
                    #del wstate[idk]
                    wstate[idk] = FALSE_TUP
                if from_sensor and bit_is_set(config.sensing_mode, REPORT_POS_SENSING):
                    config.reporter.report_sensing(self, k, v, negative=False, active_sense=active_sense)
        self.last_state = wstate.copy()

    def plan_contains_expandable_assertions(self, plan, task):
        planner = config.planner
        self.log("checking expandable replan conditions", file=config.debug, screen=False)
        executable_actions = plan.get_level1_actions()
        if any(a.is_assertion() for a in executable_actions):
            # this should not happen - but does, occasionally. no clue why.
            # anyway it's a clear case for replanning
            return True
        expandable_assertions = self.expandable_assertions()
        plan_actions = set(str(n.action) for n in plan.nodes())
        #print "planac", repr(plan_actions)
        #print "exass", repr(expandable_assertions)
        expandable_in_plan = plan_actions & expandable_assertions
        if expandable_in_plan:
            self.log("plan is invalid because replan conditions of the following actions are satisfied:", file=config.debug, screen=False)
            self.log(expandable_in_plan, file=config.debug, screen=False)
        else:
            self.log("no expandable assertions found.")
        return bool(expandable_in_plan)

    def plan_does_not_achieve_goal(self, plan, task):
        planner = config.planner
        monitor_result = planner.monitor_plan(plan, task, self.pddl_domain_fn())
        if not monitor_result.is_executable:
            self.log("plan is invalid because it is no longer executable", file=config.debug, screen=False)
        if not monitor_result.reaches_goal_state:
            self.log("plan is invalid because it no longer achieves the goals.", file=config.debug, screen=False)
        plan_invalid = not monitor_result.is_executable or not monitor_result.reaches_goal_state
        return plan_invalid
    
    def plan_has_become_invalid(self, plan, task):
        if plan is None:
            return True
        # do some invalidity checks in the given order. stop as soon as a check succeeds.
        check_order = ["assertions", "validity"]
        #check_order = ["validity", "assertions"]
        for check in check_order:
            if check == "assertions":
                plan_invalid = self.plan_contains_expandable_assertions(plan, task)
            if check == "validity":
                plan_invalid = self.plan_does_not_achieve_goal(plan, task)
            if plan_invalid:
                return check
        return False 

    def add_possible_command(self, command, stop_and_raise=False):
        self._next_commands.append(command)
        if stop_and_raise:
            raise command

    def react_to_no_plan_found(self, stop_and_raise=True):
        # TODO: somehow reflect this in the stats???
        tmpfn = os.path.basename(syscall.last_output_fn)
        if not self._next_commands:
            self.log("found NO new plan; agent is going to PASS. Planner output stored in %s." % tmpfn, vlevel=3)
        self.add_possible_command(commands.WaitCommand(self), stop_and_raise)

    def react_to_goal_achieved(self):
        self.log("has reached goal", vlevel=3)
        self.add_possible_command(commands.DoneCommand(self), stop_and_raise=True)

    def update_task(self, new_problem):
        self.problem = new_problem
        self.world_state = new_problem.init_state

    def react_to_execution(self, command, successful):
        #TODO: make this a method of command!!!
        if successful:
            self.information_state.command_was_executed(command)
            if isinstance(command, commands.Request):
                sg_name = command.sg_info.subgoal_name
                self.requests_made[command.mapl_action]
                #print "made new request '%s'. old ones: %s" % (action, [str(a) for a in self.requests_made])
            else:
                if isinstance(command, commands.MAPLCommand):
                    self.log("Executing command '%s' in agent's world model, too" % command)
                    action = command.mapl_action
                if isinstance(command, (commands.PhysicalAction, commands.TellValue)):
                    if self.planning_memory.action_in_current_plan(action):
                        self.planning_memory.advance(action)
                    self.execute_plan([action])  # simulate execution in agent's own world model
                    self.world_state.update(action.rw_description.written_facts)
                    self.problem.init_state.update(self.world_state)
                    #print "curwstate", self.world_state.to_mapl_factlist()
        else:
            self.log("Failed to execute command '%s'" % command, vlevel=2)
            # TODO: react to unsuccessful execution

    def accepted_requests(self):
        return self.requests_made  # assume all requests are accepted

    def determine_open_requests(self):
        return self.requests_made - self.accepted_requests()

    def agent_is_fully_committed(self, agt, plan):
        accepted_actions = self.requests_made  # TODO: implied actions should be in there, too
        planned_actions = plan.actions_by(agt)
        return set(accepted_actions) == set(planned_actions)

#     def determine_agent_commitments(self, plan):
#         """check if the requests made to (and accepted by) other agents are consistent with
#         your current plan so that no further negotiations are necessary"""
#         involved_agents = set(a.agent for a in self.requests_made)
#         open_requests = self.determine_open_requests()
#         uncommited_agents = set(a.agent for a in open_requests)
#         commited_agents = involved_agents - uncommited_agents
#         wstate = self.world_state
#         old_commitments = [svar for svar, val in wstate.items_by_svar(COMMITED_TO_PLAN)]
#         for svar in old_commitments:
#             del wstate[svar]
#         for agt in commited_agents:
#             svar = state.StateVariable([COMMITED_TO_PLAN, agt])
#             wstate[svar] = TRUE_TUP
#             self.set_mapl_problem_dirty()
#             self.log("%s is commited to my plan." % agt)

    def determine_agent_commitments(self, plan):
        """check if the requests made to (and accepted by) other agents are consistent with
        your current plan so that no further negotiations are necessary"""
        involved_agents = plan.agents_involved()
        uncommited_agents = set(agt for agt in involved_agents if not self.agent_is_fully_committed(agt, plan))
        commited_agents = involved_agents - uncommited_agents
        wstate = self.world_state
        old_commitments = [svar for svar, val in wstate.items_by_svar(COMMITED_TO_PLAN)]
        for svar in old_commitments:
            del wstate[svar]
        for agt in commited_agents:
            svar = state.StateVariable([COMMITED_TO_PLAN, agt])
            wstate[svar] = TRUE_TUP
            self.set_mapl_problem_dirty()
            self.log("%s is commited to my plan." % agt)


    def select_best_requests(self, nodes):
        if config.request_subplans:
            # select the latest for each agent
            maxlevel = defaultdict(int)
            for n in nodes:
                agt = n.action.agent
                maxlevel[agt] = max(maxlevel[agt], n.level)
            best_requests = [n.action for n in nodes if n.level == maxlevel[n.action.agent]]
        else:
            best_requests = [n.action for n in nodes if n.level == 1]
        return set(best_requests)

    def react_to_subgoal_achieved(self, sg_name):
        self.log("Subgoal %s was achieved" % sg_name, vlevel=2)
        if sg_name not in self.temp_goals:
            return
        sg_info = self.temp_goals[sg_name]
        sg_name, subgoal_op, command, requesting_agent, executing_agent = sg_info.as_tuple()
        del self.temp_goals[sg_name]
        self.log("removing subgoal %s" % sg_name, vlevel=2)
        
        self.recompute_goal_formula()
        if self == requesting_agent and bit_is_set(config.ack_mode, ACK_SG_ACHIEVEMENT):
            command = command.change_to(commands.ThankSubgoalAchieved, False, self.simulator)
            last_command = self.information_state.last_command_sent()
            if last_command:
                if str(last_command.mapl_action) == str(command.mapl_action):
                    command.report_modifiers["long"] = False
            del self.requests_made[command.mapl_action]
            self.add_possible_command(command)


    def execute_virtual_action(self, action):
        self.log("  (executing virtual action '%s')" % action, vlevel=3, info=INFO_HIDDEN_ACTIONS)
        plan = self.current_plan
        aname = action.description
        execution_successful = self.execute_plan([action], vlevel=3)
        self.planning_memory.advance(action)
        self.log("removing action %s" % action, vlevel=5)
        plan.remove(action)
        if aname.startswith(ACHIEVE_SG_PREFIX):
            op_name = action.operator
            sg_name = op_name.split("_")[-1]
            assert sg_name.startswith("sg")
            self.react_to_subgoal_achieved(sg_name)

    def serendipity_check(self, action):
        """checks whether an action currently scheduled for execution is
        unnecessary because its effects have already been achieved"""
        #print "serendipity check for action", action
        wstate = self.world_state
        effects = action.rw_description.written_facts
        def effect_holds_already(svar, val, wstate):
            assert isinstance(svar, state.StateVariable)
            if not svar.is_knowledge_proposition():
                return val == wstate.get(svar)
            svar = svar.base_of_knowledge_proposition(self.name)
            if val == TRUE_TUP:
                return wstate.get(svar) is not None
            elif val == FALSE_TUP:
                return wstate.get(svar) is None
        is_serendipitous = all(effect_holds_already(svar, val, wstate) for svar, val in effects.items())
        if is_serendipitous:
            self.log("action '%s' is serendipitous." % action, vlevel=2)
            self.current_plan = None
            self.planning_memory.advance(action)

    def currently_executable_actions(self):
        """iterate over self.current_plan and return level one actions.
        The plan may change during the iteration, so this is a generator
        who taskes the updated plan into account."""
        checked_actions = set()
        while True:
            actions = self.get_current_plan().get_level1_actions()
            unchecked_actions = set(actions) - checked_actions
            if unchecked_actions:
                action = iter(unchecked_actions).next()
                checked_actions.add(action)
                yield action
            else:
                return

    def update_current_plan(self):
        """execute 'virtual' actions, detect serendipities and/or actions executed by other agents"""
        self.current_plan = self.get_current_plan()
        self.log("\ncurrent plan before virtual-action/serendipity check:\n%s" % self.current_plan, vlevel=5)
        try:
            for action in self.currently_executable_actions():
                if action.is_assertion():
                    self.current_plan = None
                    return
                if action.is_hidden() or action.is_goal_operator():
                    self.execute_virtual_action(action)
                elif action.is_sensing_action():
                    if not config.execute_sensing_actions:
                        self.execute_virtual_action(action)
                    else:
                        return
                else:
                    self.serendipity_check(action)
                self.current_plan = self.planning_memory.current_plan()
        finally:
            # don't catch Command exceptions here, but make sure the current plan is updated
            self.current_plan = self.planning_memory.current_plan()
            self.log("\ncurrent plan after virtual-action/serendipity check:\n%s" % self.current_plan, vlevel=5)
        
    def monitor_and_replan(self):
        self.set_mapl_problem_dirty()
        task = self.get_mapl_problem(file=True)
        plan = self.current_plan = self.get_current_plan()
        self.determine_agent_commitments(plan)
        plan_invalid = self.plan_has_become_invalid(plan, task)
        self.log("\ncurrent plan (invalid: %s):\n%s" % (plan_invalid, plan), vlevel=4)
        if plan_invalid:
            self.log("replanning necessary!", vlevel=3)
            starttime = time.time()
            ff_plan, action_descriptions = config.planner.find_plan(task, self.cpddl_domain_fn())
            plan = plans.extract_plan(ff_plan, action_descriptions, self, make_PO=True)
            duration = time.time() - starttime
            self.stats.plans += 1
            self.stats.planning_time += duration
            if plan is None:
                call = syscall.last_call
                self.log("No plan was found by the following PLANNER call:\n%s" % call, vlevel=3)
                ccp_state = idl_classes.CCPState.from_cp(goal_unachievable=True, 
                                                         goal_achieved=False, changed_plan=True, plan=None)
                return ccp_state
            tmpfn = os.path.basename(syscall.last_output_fn)
            self.log("found plan in %.1f secs.  planner output stored in %s." % (round(duration,2), tmpfn), vlevel=3)
            self.log("new plan:\n%s" % plan, vlevel=3)
            use_new_PM_for_new_plan = True
            if use_new_PM_for_new_plan:
                self.planning_memory = plans.PlanningMemory(self)
            self.planning_memory.add_plan(plan)
##             self.execute_virtual_actions(plan)
            self.update_current_plan()
            plan = self.current_plan
        else:
            self.stats.plans_kept += 1
            self.log("old plan is still valid.", vlevel=5)
        plan.write_dot_file(self.unique_dot_file_name())
        self.planning_memory.write_dot_file(self.unique_dot_file_name(pm=True))
        ccp_state = idl_classes.CCPState.from_cp(goal_unachievable=plan is None, goal_achieved=plan.is_empty(), 
                                     changed_plan=plan_invalid, plan=plan)
        return ccp_state

    def build_request(self, command):
        requesting_agent = self
        executing_agent_name = command.mapl_action.agent
        subgoal_name, subgoal_op = self.create_temporary_subgoal(command, self.mapl_domain)
        sg_info = SubgoalInfo(subgoal_name, subgoal_op, command, requesting_agent, executing_agent_name)
        return commands.Request(sg_info)

    def select_best_negotiation_command(self, plan, negotiations_planned):
        if not negotiations_planned:
            return None
        self.log("Selecting negotiation command...", vlevel=10)
        agent2naction = dict((a.arguments[0],a) for a in negotiations_planned)
        if config.only_one_request and self.requests_made:
            self.log("Only one request allowed (currently: %s)." % self.requests_made.iter().next(), vlevel=10)
            return None
        request_nodes = [n for n in plan.nodes() if n.action.agent.lower() != self.name.lower()]
        requests = self.select_best_requests(request_nodes)
        requests -= set(self.requests_made)
        if not requests:
            self.log("No negotiation command was selected.", vlevel=10)
            return
        requests = list(requests)
        random.shuffle(requests)
        for action in requests:
            command = commands.MAPLCommand.from_ground_action(action, self)
            request = self.build_request(command)
            negotiation_action = agent2naction[action.agent]
            self.planning_memory.advance(negotiation_action)
            self.add_possible_command(request)

    def select_best_physical_action(self, executable_actions):
        acceptable_action_types = set([PHYS_ACTION, OTHER_SPEECH_ACT])
        if config.execute_sensing_actions:
            acceptable_action_types.add(SENSING_ACTION)
        def acceptable_action(action):
            #print self.name.lower(), a.agent, a, a.action_type
            if a.action_type not in acceptable_action_types:
                return False
            if a.agent != self.name.lower():
                return False
            return True
        phys_actions = [a for a in executable_actions if acceptable_action(a)]
        if not phys_actions:
            self.log("None of these physical actions seems to be executable: %s" % 
                     it2str(executable_actions), vlevel=2)
            return
        self.log("Selecting physical action...", vlevel=10)
        random.shuffle(phys_actions)
        for action in phys_actions:
            action = random.choice(list(phys_actions))
            command = commands.MAPLCommand.from_ground_action(action, self)
            self.add_possible_command(command)

    def select_best_action(self, plan):
        if plan is None:
            self.react_to_no_plan_found()
        elif plan.is_empty():
            self.react_to_goal_achieved()
        candidate_actions = plan.get_level1_actions()
        # bug here: actions relying on axioms might appear too early in the plan
        # (because the causal relationship with their (indirect) provider is not detected)
        # workaround: check if level1 actions are really executable in the current state
        candidate_actions = set(a for a in candidate_actions if str(a) in self.possible_actions())
        negotiations_planned = [a for a in candidate_actions if a.operator == NEGOTIATE_PLAN_OP]
        #negotiations_planned = [a for a in candidate_actions if not self.same_name(a.agent) or a.operator == NEGOTIATE_PLAN_OP]
        self.select_best_negotiation_command(plan, negotiations_planned)
        self.select_best_physical_action(candidate_actions)
       
    def adapt_command (self, command):
        #print command, "type", command.action_type
        if isinstance(command, commands.Request):
            command.sg_info.command = self.adapt_command(command.sg_info.command)
        elif command.action_type == UNDEFINED:
            if command.mapl_action.is_tell_val():
                svar, val = self.determine_svar_val(command)
                newcommand = commands.TellValue(self, command.mapl_action.arguments[0], svar, val)
                newcommand.mapl_action = command.mapl_action
                command = newcommand
            else:
                if command.action_type == PHYS_ACTION:
                    command = command.copy()
                    command.__class__ = commands.PhysicalAction
        return command
 
    def update_after_command_selection(self, command):
        """ update stats and other data after command has been chosen to be executed next"""
        self.planning_memory.write_dot_file(self.unique_dot_file_name(pm=True))
        if isinstance(command, commands.PassCommand):
            self.stats.passed += 1
            self.log("passes", vlevel=3)
        else:
            self.log("sends command (%s)" % command, vlevel=2)
            self.information_state.command_was_sent(command)
            self.stats.commands += 1

    def received_request (self, msg):
        """ react to a newly received request"""
        assert isinstance(msg, commands.Request)
        hearer = msg.hearer().lower()
        myself = self.name.lower()
        assert hearer == myself, "%s != %s" % (hearer, myself)
        is_directly_satisfiable = str(msg.mapl_action) in self.possible_actions()
        if bit_is_set(config.ack_mode, NO_ACK_SG_ACCEPTANCE_IF_DIRECT_ACHIEVEMENT) and is_directly_satisfiable:
            tmp_plan = plans.POPlan(self)
            tmp_plan.add(plans.PO_Node(msg.mapl_action, planning_agent=self))
            self.planning_memory.add_plan(tmp_plan)
            assert msg.mapl_action.agent.lower() == self.name.lower()
            command = commands.MAPLCommand.from_ground_action(msg.mapl_action, self)
            self.add_possible_command(command)
        sg_info = msg.sg_info
        individually_satisfiable = self.check_achievability(sg_info.goal_condition())
        consistent_with_old_goals = True  #TODO: check joint achievability here
        if individually_satisfiable:
            self.add_temporary_subgoal(msg.sg_info)
            if (config.ack_mode & ACK_SG_ACCEPTANCE):
                msg = msg.change_to(commands.AckSubgoalAcceptance, True, self.simulator)
                self.add_possible_command(msg)
        else:
            if (config.ack_mode & ACK_SG_ACCEPTANCE):
                msg = msg.change_to(commands.AckSubgoalRejection, True, self.simulator)
                self.add_possible_command(msg)

    def keep_turn(self, last_commmand, execution_result):
        # TODO: execution_result etc should also be in the information state
        last_type = type(last_commmand)
        if last_type == commands.AckSubgoalAcceptance:
            return True
        if last_type == commands.ThankSubgoalAchieved:
            return True
        return False

    def handle_incoming_messages(self):
        messages = self.messages_received
        while messages:
            msg = messages.popleft()
            self.log("received message '%s'" % msg, vlevel=2)
            msg.received(self)
            
    def compute_next_commands(self):
        """ Selects a list of possible next commands.  Their order
        describes internal priorities of the CCP algorithm, but they are
        all executable, so this order can safely be ignored."""
        self._next_commands = []
        try:
            # sometime during this block a Command exception will be thrown
            self.handle_incoming_messages()
            self.update_current_plan()
            ccp_state = self.monitor_and_replan()
            plan = ccp_state.plan
            self.select_best_action(plan)
            self.react_to_no_plan_found(stop_and_raise=True)
        except commands.Command, command:
            next_commands = self._next_commands
            next_commands = [self.adapt_command(cmd) for cmd in next_commands]
            cmd_strs = [str(cmd) for cmd in next_commands]
            self.log("Found %d next possible commands: %s" % (len(next_commands), cmd_strs), vlevel=2)
            return next_commands
        assert False, "A command exception should have been raised by now."
