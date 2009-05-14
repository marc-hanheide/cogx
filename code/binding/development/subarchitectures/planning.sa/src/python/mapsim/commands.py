#! /usr/bin/env python
# -*- coding: latin-1 -*-

from copy import deepcopy
from copy import copy as shallowcopy

import config
from config import log
from constants import *
from state import StateVariable
from utils import *
import plans

EXEC_SUCCESS = True
VERBALIZE = True

class Command(Exception):
    def __init__(self, agent=None, operator=None, arguments=None):
        self.agent = agent
        self.operator = operator
        if arguments is None:
            arguments = []
        self.arguments = arguments
        self.report_modifiers = {}   

    def get_action_type(self):
        return UNDEFINED
    action_type = property(get_action_type)

    def get_arguments(self):
        # to be overwritten or extended by subclasses
        return [self.agent]+ self.arguments

    def reporter_template(self):
        return "DOES_TMPL"

    def is_speech_act(self):
        return False

    def execute(self, simulator=None):
        pass

    def __hash__(self):
        return hash(self.operator)

    def __eq__(self, obj):
        return isinstance(obj,Command) and self.operator == obj.operator

    def __str__(self):
        parts = map(str, [self.operator, self.agent]+list(self.arguments))
        return " ".join(parts)


class StopSimulationCommand(Command):
    def __init__ (self, reason, success):
        self.reason = reason
        self.success = success
    def __str__(self):
        return "StopSimulationCommand:'%s'" % self.reason


class PassCommand(Command):
    action_type = PHYS_ACTION    
    def __init__(self, agent):
        raise NotImplementedError, "Only subclasses of PassCommand should be used"
    def is_done(self):
        # to ease distinguishing
        return isinstance(self, DoneCommand)

class WaitCommand(PassCommand):
    def __init__(self, agent):
        Command.__init__(self, agent=agent, operator="wait")

class YieldTurnCommand(PassCommand):
    def __init__(self, agent):
        Command.__init__(self, agent=agent, operator="yield_turn")

class DoneCommand(PassCommand):
    def __init__(self, agent):
        Command.__init__(self, agent=agent, operator="done")

    
class MAPLCommand(Command):
    def __init__(self, agt, operator, arguments=[]):
        Command.__init__(self, agt)
        self.mapl_action = plans.Action(agt, operator, arguments, planning_agent=agt)
    
    def __str__ (self):
        msg_type = str(self.__class__.__name__)
        args = " ".join(self.arguments)
        msg = str(self.mapl_action)
        return "%(msg_type)s %(args)s: %(msg)s" % locals()

    def copy(self):
        return deepcopy(self)

    def __deepcopy__(self, memo):
        from agent import Agent
        assert isinstance(agt, Agent), "%s is of type %s instead of Agent in command %s" % (agt, type(agt), self)
        agt = shallowcopy(self.agent)
        operator = shallowcopy(self.mapl_action.operator)
        arguments = shallowcopy(self.mapl_action.arguments)
        new_cmd = MAPLCommand(agt, operator, arguments)
        return new_cmd
        
    @classmethod
    def from_ground_action(cls, action, agt):
        a = action.copy()
        c = MAPLCommand(agt, a.operator, a.arguments)
        c.mapl_action = a
        c.adapt_class()
        return c
        
    def adapt_class(self):
        classes = {PHYS_ACTION : PhysicalAction, REQUEST : Request,
                   SENSING_ACTION: SensingAction, OTHER_SPEECH_ACT : MAPLCommand}
        self.__class__ = classes[self.mapl_action.action_type]

    def get_action_type(self):
        return self.mapl_action.action_type

    def executing_agent_name(self):
        return self.mapl_action.agent
    
    def is_request(self):
        return False

    def is_tell_val(self):
        return self.mapl_action.is_tell_val()


class SensingAction(MAPLCommand):
    """ describes actions that effect the knowlegde of agents, but based on
        the state of the physical world."""
    action_type = SENSING_ACTION    
    def reporter_template(self):
        return None
    
class PhysicalAction(MAPLCommand):
    """ describes actions that act on the physical world, i.e. the simulation"""
    action_type = PHYS_ACTION    

class ActiveSensingAction(SensingAction):
    """ sensing action that is executed deliberately by an agent """

class PassiveSensingAction(SensingAction):
    """ sensing action that is triggered automatically by the environment """

class SpeechAct(MAPLCommand):
    """ a speech act is an action that is executed deliberately by an agent
    and changes the mental state (beliefs, goals) of another one."""

    action_type = OTHER_SPEECH_ACT

    def __init__ (self, speaker, hearer, operator=None):
        MAPLCommand.__init__(self, agt=speaker, operator=operator)
        self.the_speaker = speaker
        self.the_hearer = hearer

    def __deepcopy__(self, memo):
        the_speaker = shallowcopy(self.the_speaker)
        the_hearer = shallowcopy(self.the_hearer)
        operator = shallowcopy(self.mapl_action.operator)
        new_cmd = self.__class__(the_speaker, the_hearer, operator)
        return new_cmd    

    def get_arguments(self):
        return [str(self.speaker()), str(self.hearer())]

    def is_speech_act(self):
        # return self.is_request() or self.is_tell_val()
        return True
    
    def speaker(self):
        return self.the_speaker

    def hearer(self):
        return self.the_hearer

    def execute(self, simulator):
        """ Attention: simulator must be provided here because we have to
        resolve the reference to the hearer in order to send the message to
        him."""
        hearer = simulator.get_agent(self.hearer())
        # make a copy and put it into receiver's message queue
        hearer_speech_act = self.copy()
        #hearer_speech_act.the_hearer = hearer
        hearer.messages_received.append(hearer_speech_act)

    def received(self, agt):
        """ Dispatching method for speficic speech acts.  Subclasses will
        (most likely) hand this back to special methods of the agent."""
        log("In SpeechAct.received(). This should be overridden!", vlevel=2)
        raise NotImplementedError, "Speech act of type %s does not implement received() method" % type(self)

    def change_to(self, new_class, swap_interlocutors, simulator):
        new_command = self.copy()
        new_command.__class__ = new_class
        if swap_interlocutors:
            new_command.swap_interlocutors(simulator)
        return new_command

    def swap_interlocutors(self, simulator):
        old_speaker = self.speaker().name
        if simulator:
            old_hearer = simulator.get_agent(self.hearer())
        else:
            old_hearer = self.hearer()
        self.the_hearer, self.the_speaker = old_speaker, old_hearer
        self.agent = self.the_speaker

    def reporter_template(self):
        return "$speaker: communicates with $hearer."

    def __str__ (self):
        speaker = self.speaker()
        hearer = self.hearer()
        msg_type = str(self.__class__.__name__)
        msg = MAPLCommand.__str__(self)
        return "%(msg_type)s[%(speaker)s->%(hearer)s]: %(msg)s" % locals()
        

class TellValue(SpeechAct):
    """ tell another agent the value of a state variable """
    def __init__ (self, speaker, hearer, svar, value):
        SpeechAct.__init__(self, speaker, hearer)
        self.svar = svar
        self.value = value

    def __deepcopy__(self, memo):
        new_cmd = self.__class__(self.speaker(), self.hearer(), self.svar, self.value)
        new_cmd.mapl_action = self.mapl_action.copy()
        return new_cmd

    def execute(self, simulator):
        self.speaker().execute_plan([self.mapl_action], realize_k_effects=True, simulator=simulator)
        SpeechAct.execute(self, simulator)

    def received(self, agt):
        log("receiving TellVal(). All K-effects currently handled by speaker!", vlevel=2)
        mapl_action = self.mapl_action
        if mapl_action in agt.requests_made:
            sg_name = agt.requests_made[mapl_action]
            agt.react_to_subgoal_achieved(sg_name)

    def reporter_template(self):
        return "DOES_TELL_VAL_TMPL"

    def __str__ (self):
        speaker = self.speaker()
        hearer = self.hearer()
        msg_type = str(self.__class__.__name__)
        msg = "%s=%s" % (self.svar, self.value)
        return "%(msg_type)s[%(speaker)s->%(hearer)s]: %(msg)s" % locals()


class SubgoalRelatedCommand(MAPLCommand):
    """ any command that relates to a subgoal and stores information about it """
    def __init__ (self, sg_info):
        self.__dict__.update(sg_info.command.__dict__)
        self.sg_info = sg_info


class SubgoalRelatedSpeechAct(SpeechAct, SubgoalRelatedCommand):
    def __init__ (self, sg_info):
        SubgoalRelatedCommand.__init__(self, sg_info)
        SpeechAct.__init__(self, sg_info.requesting_agent, sg_info.executing_agent)
        self.mapl_action = sg_info.command.mapl_action

    def __deepcopy__(self, memo):
        new_cmd = self.__class__(self.sg_info.copy())
        return new_cmd



class Request(SubgoalRelatedSpeechAct):
    """ a speech act for requesting another agent to do something """

    action_type = REQUEST

    def execute(self, simulator):
        simulator.verbalize_request(self)
        SpeechAct.execute(self, simulator)

    def received(self, agt):
        agt.received_request(self)

    def is_request(self):
        return True

    def requested_command (self):
        return self.sg_info.command

    def requesting_agent_name(self):
        return str(self.the_speaker)

    def reporter_template(self):
        if isinstance(self.requested_command(), TellValue):
            return "REQUEST_TELL_VAL_TMPL"
        return "REQUEST_TMPL"

    def __str__ (self):
        speaker = self.speaker()
        hearer = self.hearer()
        msg_type = str(self.__class__.__name__)
        msg = MAPLCommand.__str__(self) + " (%s)" % str(self.requested_command().__class__.__name__)
        return "%(msg_type)s[%(speaker)s->%(hearer)s]: %(msg)s" % locals()

    
    
class AckSubgoalAcceptance(SubgoalRelatedSpeechAct):
    """ speech act for accepting a request """
    def reporter_template(self):
        if (config.ack_mode & ACK_SG_ACCEPTANCE):
            return "ACK_SG_ACCEPT_TMPL"
    def received(self, agt):
        self.sg_info.state = SubgoalState.SG_ACCEPTED_BY_OTHER

class AckSubgoalRejection(SubgoalRelatedSpeechAct):
    """ speech act for declining a request """
    def reporter_template(self):
        if not (config.ack_mode & ACK_SG_ACCEPTANCE):
            return None
        is_question = True
        self.report_modifiers["question"] = is_question
        return "ACK_SG_REJECT_TMPL"

class ThankSubgoalAchieved(SubgoalRelatedSpeechAct):
    """ speech act for acknowledging and thanking another agent who has
    achieved a subgoal as requested """
    def reporter_template(self):
        if (config.ack_mode & ACK_SG_ACHIEVEMENT):
            return "THANKS_TMPL"
    def received(self, agt):
        sg_name = self.sg_info.subgoal_name
        log("%s was informed that subgoal %s was achieved" % (agt, sg_name), vlevel=2)
        agt.react_to_subgoal_achieved(sg_name)

   
