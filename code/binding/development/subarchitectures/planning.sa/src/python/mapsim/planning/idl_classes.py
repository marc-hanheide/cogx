""" 
Helper classes that extend the classes that were generated automatically
from the Planner.idl specification.  The helper classes provides some
additional utilities that are helpful when setting up the data, but should
nevertheless should go through CORBA without problems.
"""

import omniORB, _omnipy

import Planner as PlannerIDL
from Planner import GOAL_ACHIEVABLE, GOAL_UNACHIEVABLE, GOAL_ACHIEVED, OLD_PLAN_KEPT, CHANGED_PLAN
from Planner import NO_MODALITY, K_MODALITY
from Planner import ExecutionState

import mapl

modalities = {"":NO_MODALITY, "k":K_MODALITY}

DEBUG = False

def class_name(obj):
    return str(obj.__class__.__name__)

def is_cast_obj(obj):
    # unfortunately, this is not so easy to find out, because omniorbpy
    # doesn't give CORBA objects a common subclass.  So, let's try this:
    return hasattr(obj, "_NP_RepositoryId")


def convert2extended(cast_obj):
    """turns a CORBA struct into an extended class (as defined in this module)
    with member functions and whatever else we might need for convenience
    """
    if isinstance(cast_obj, (list,tuple)):
        for obj in cast_obj:
            convert2extended(obj)
        return
    if not is_cast_obj(cast_obj):
        if DEBUG:  "no need to convert", class_name(cast_obj)
        return cast_obj
    if DEBUG:  "converting a", class_name(cast_obj)
    for field, val in cast_obj.__dict__.items():
        if DEBUG:  "%s : '%s' (%s)" % (field, val, type(val))
        convert2extended(val)
    cast_class = class_name(cast_obj)
    extended_class = globals()[cast_class]
    cast_obj.__class__ = extended_class

def convert2stub(cast_obj):
    """turns an extended class back into a CORBA stub."""
    if isinstance(cast_obj, (list,tuple)):
        for obj in cast_obj:
            convert2stub(obj)
        return
    assert cast_obj is not None
    if not is_cast_obj(cast_obj):
        if DEBUG:  "no need to convert", class_name(cast_obj)
        return cast_obj
    if DEBUG:  "converting a", class_name(cast_obj)
    for field, val in cast_obj.__dict__.items():
        if DEBUG:  "%s : '%s' (%s)" % (field, val, type(val))
        convert2stub(val)
    cast_class = class_name(cast_obj)
    stub_class = PlannerIDL.__dict__[cast_class]
    cast_obj.__class__ = stub_class


class ObjectDeclaration(PlannerIDL.ObjectDeclaration):
    pass

class Fact(PlannerIDL.Fact):
    @classmethod
    def from_MAPL(cls, svar, val):
        modality_str, agt, new_svar = svar.split_modal_pddl_representation() 
        if isinstance(val,tuple):
            val = " ".join(val)
        #print "fact:", new_svar, ":", val
        mod = modalities.get(modality_str)
        if mod is None:
            print "unknown modality '%s' will be treated as part of svar '%s'" % (modality_str, svar)
            new_svar = svar
        return cls(mod, agt, new_svar.name, new_svar.args, val)
    def to_MAPL(self):
        import state
        factstr = state.mapl_str2assignment(self.mapl_str())
        return factstr
    def mapl_str(self):
        if self.value:
            fact_str = "(%s %s : %s)" % (self.name, " ".join(self.arguments), self.value)
        else:
            fact_str = "(%s %s)" % (self.name, " ".join(self.arguments))
        if self.modality == NO_MODALITY:
            modal_fact_str = fact_str
        elif self.modality == K_MODALITY:
            modal_fact_str =  "(K %s %s)" % (self.agent, fact_str)
        else:
            print "Unknown modality!"
        #print "modal_fact_str", modal_fact_str
        return modal_fact_str
    def __str__(self):
        return "%s: (%s %s : %s)" % (class_name(self), self.name, " ".join(self.arguments), self.value)
    def __repr__(self):
        return str(self)

class GroundAction(PlannerIDL.GroundAction):
    @classmethod
    def from_MAPL(cls, mapl_action):
        name = str(mapl_action.operator)
        args = [str(arg) for arg in mapl_action.arguments]
        ground_action = cls(name, args)
        ground_action.mapl_action = mapl_action
        return ground_action
    @classmethod
    def make_dummy(cls, task_id, action_name="dummy_ground_action"):
        return cls(action_name, [task_id])
    def __str__(self):
        msg = "%s %s" % (self.name, " ".join(self.args))
        #s = "%s: %s" % (class_name(self), msg)
        return msg

class Command(PlannerIDL.Command):
    @classmethod
    def from_MAPL(cls, mapl_command, task_id=""):
        cmd_type = str(mapl_command.__class__.__name__)
        cmd_args = [str(arg) for arg in mapl_command.get_arguments()]
        try:
            ground_action = GroundAction.from_MAPL(mapl_command.mapl_action)
        except AttributeError:
            ground_action = GroundAction.make_dummy(task_id)
        cmd_idl = cls(cmd_type, cmd_args, ground_action, task_id)
        cmd_idl.mapl_command = mapl_command
        return cmd_idl
    def __str__ (self):
        msg_type = self.type
        args = " ".join(self.cmd_args)
        msg = str(self.mapl_action)
        return "%(msg_type)s %(args)s: %(msg)s" % locals()
    def __eq__(self, obj):
        return str(self) == str(obj)
    def __hash__(self):
        return hash(str(self))

class PlanningTask(PlannerIDL.PlanningTask):
    @classmethod
    def from_MAPL(cls, problem, task_id=""):
        """ convert a MAPL problem to IDL structure """
        objects = [PlannerIDL.ObjectDeclaration(name=o.name, type=o.type) for o in problem.objects]
        facts = [Fact.from_MAPL(svar, val) for svar, val in problem.init_state.items()]
        goals = [problem.goal.mapl_str()]
        task = cls(task_id, str(problem.planning_agent), objects, facts, goals, problem.domain_name, "")
        return task
    def parts(self):
        return (self.task_id, self.planning_agent, self.objects, 
                self.facts, self.goals, self.domain_name, self.domain_fn)
    def to_MAPL(self):
        mapl_prob_text = build_mapl_problem(self)
        print "mapl_prob_text:", mapl_prob_text
        problem = mapl.load_mapl_task(mapl_prob_text)
        return problem
    def __str__(self):
        l = []
        l.append("Task ID: %s" % self.task_id)
        l.append("Objects:")
        for obj in self.objects:
            l.append("   %s - %s" % (obj.name, obj.type))
        l.append("Facts:")
        for fact in self.facts:
            l.append("   %s" % fact)
        l.append("Goals:")
        for goal in self.goals:
            l.append("   %s" % goal)
        return "\n".join(l)


def build_mapl_problem(cast_task):
    task_template =  """(define (problem %s)\n(:domain %s)\n""" \
        """(:objects\n%s\n)\n(:init\n%s\n)\n\n(:goal \n%s\n))"""
    task_ID, planning_agent, objects, facts, goals, domain_name, domain_fn = cast_task.parts()
    objects = "\n".join("  %s - %s" % (obj.name, obj.type) for obj in objects)
    facts = "\n".join("  %s" % fact.mapl_str() for fact in facts)
    if len(goals) == 1:
        goal = goals[0]
    else:
        goal = "(and\n%s)" % "\n".join(goals)
    task_name = "%s_task" % domain_name
    task = task_template % (task_name, domain_name, objects, facts, goal)
    return task.splitlines()

class CCPState(PlannerIDL.CCPState):
    @classmethod
    def from_cp(cls, goal_unachievable, goal_achieved, changed_plan, plan, info="dummy"):
        if goal_achieved:
            execution_state = GOAL_ACHIEVED
        elif goal_unachievable:
            execution_state = GOAL_UNACHIEVABLE
        else:
            execution_state = GOAL_ACHIEVABLE
        if changed_plan:
            planning_state = CHANGED_PLAN
        else:
            planning_state = OLD_PLAN_KEPT
        info = PlannerIDL.CCPInformation(info)
        ccp_state = cls(execution_state, planning_state, info)
        ccp_state.plan = plan
        return ccp_state

class AckGoal(Command):
    def __init__(self, task_id):
        cmd_type = str(self.__class__.__name__)
        cmd_args = [task_id]
        ground_action = PlannerIDL.GroundAction(cmd_type, cmd_args)
        Command.__init__(self, cmd_type, cmd_args, ground_action, task_id)

class AckGoalAccepted(AckGoal):
    pass
        
class AckGoalRejected(AckGoal):
    pass        

class AckGoalHoldsAlready(AckGoal):
    pass        

