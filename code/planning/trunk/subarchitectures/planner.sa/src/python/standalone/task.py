import constants
import state
import mapl.types as types
import mapl.predicates as predicates
from utils import Enum

PlanningStatusEnum = Enum("TASK_CHANGED", "RUNNING", "PLAN_AVAILABLE", "PLANNING_FAILURE", "INTERRUPTED")    

class Task(object):
    """
    """

    def __init__(self, taskID=0):
        """Initialise public and private fields."""
        # public
        self.taskID = taskID
        self.planner = None  # is set by planner.register_task()
        # general domain ontology
        self._domain_name = None
        self._requirements = None
        self._type_tree = None
        self._state_variables = None
        self._constants = None
        self._operators = None
        self._axioms = None
        self._sensors = None
        # task-specific data
        self._task_name = None
        self._objects = None
        self._state = None
        self._goal = None
        self._action_blacklist = None
        self._action_whitelist = None
        self._plan = None
        # private
        self._planning_status = PlanningStatusEnum.TASK_CHANGED
        self._change_detection_activated = False

    def set_planning_status(self, status, trigger_planning=True):
        """Mark task as changed and call planner if necessary."""
        assert status in PlanningStatusEnum.values()
        self._planning_status = status
        if status == PlanningStatusEnum.TASK_CHANGED:
            print "Status of task %s was changed to %s. This may trigger planner activity." % (self.taskID, status)
        else:
            print "Status of task %s was changed to %s." % (self.taskID, status)
        if trigger_planning:
            self.update_planning()
        if status == PlanningStatusEnum.PLAN_AVAILABLE:
            print "Heureka! A plan was found:\n%s" % self._plan

    def get_planning_status(self):
        return self._planning_status

    def mark_changed(self):
        self.set_planning_status(PlanningStatusEnum.TASK_CHANGED)

    def is_dirty(self):
        """Check if task has been modified so that the plan is possibly outdated."""
        return self.get_planning_status() == PlanningStatusEnum.TASK_CHANGED

    def activate_change_dectection(self):
        """Activated automatic change detection and calling of the planner.
        Activation may lead to immediate updating of the plan if the task was modified
        while change detection was suspended."""
        self._change_detection_activated = True
        print "Change detection was activated for task %s. This may trigger planner activity." % self.taskID
        self.update_planning()
    
    def suspend_change_dectection(self):
        """Do not update the plan immediately upon changes.  This is useful if 
        several changes are known to be made in a row.  Then planning will only be 
        triggered upon calling activate_change_dectection().""" 
        self._change_detection_activated = False

    def change_detection_activated(self):
        """Is change detection currently activated?"""
        return self._change_detection_activated

    def get_state(self):
        return self._state

    def _change_task(field, new_val, update_status=True):
        """
        Changes a field in the task and marks the task as changed
        (which might trigger replanning) if necessary
        """
        old_val = self.__dict__[field]
        self.__dict__[field] = new_val
        if update_status and old_val != new_val:
            self.set_planning_status(PlanningStatusEnum.TASK_CHANGED)

    def set_state(self, new_val, update_status=True):
        self._change_task("_state", new_val, update_status) 
            
    def get_goal(self):
        return self._goal

    def set_goal(self, update_status=True):
        self._change_task("_goal", new_val, update_status) 

    def get_plan(self):
        return self._plan

    def set_plan(self, plan, update_status=True):
        self._plan = plan
        if update_status:
            if plan is None:
                self.set_planning_status(PlanningStatusEnum.PLANNING_FAILURE)
            else:
                self.set_planning_status(PlanningStatusEnum.PLAN_AVAILABLE)

    def update_planning(self):
        """If the task is registered with a planner, the plan will be updated,
        but only if the task has been modified and  if change detection is activated."""
        if self.change_detection_activated() and self.is_dirty() and self.planner:
            self.planner.continual_planning(self)       

    def pddl_domain_str(self):
        return PDDLTask.from_MAPL_task(self).pddl_domain_str()

    def pddl_problem_str(self):
        return PDDLTask.from_MAPL_task(self).pddl_problem_str()

    def load_mapl_task(self, task_file, domain_file, agent_name):
        import globals
        import mapl
        print "Loading MAPL domain."
        domain = mapl.mapl_file.load_mapl_domain(domain_file)
        self._domain_name = domain.domain_name
        self._requirements = domain.requirements
        self._type_tree = domain.type_tree
        self._state_variables = domain.predicates
        self._constants = domain.constants
        self._operators = domain.actions
        self._axioms = domain.axioms
        self._sensors = domain.sensors
        print "Loading MAPL problem."
        problem = mapl.mapl_file.load_mapl_problem(task_file)
        self._task_name = problem.task_name
        if self._domain_name:
            assert self._domain_name == problem.domain_name
        else:
            self.domain_name = problem.domain_name
        self._objects = problem.objects
        self._state = problem.init_state
        self._goal = problem.goal
        


class PDDLTask(Task):
    @classmethod
    def from_MAPL_task(cls, mapl_task):
        """ compile a MAPL task into a corresponding PDDL representation. """
        import copy
        t = copy.deepcopy(mapl_task)  # only a shallow copy. is that enough?
        t.__class__ = PDDLTask
        t.add_implicit_MAPL_types(constants.MAPL_BASE_ONTOLOGY.split())
        #t._type_tree.closure()
        for pred in constants.MAPL_BASE_PREDICATES:
            t.add_implicit_MAPL_predicate(pred)
        new_objects = types.parse_typed_list(constants.MAPL_BASE_OBJECTS.split())
        t.add_implicit_MAPL_constants(new_objects)
        t._operators = [a.translate2pddl(t._state_variables) for a in t._operators]
        return t

    def add_implicit_MAPL_types(self, mapl_type_list):
        tt = types.TypeTree.from_typed_list(mapl_type_list)
        for sup, subs in tt.items():
            for sub in subs:
                self._type_tree.add_type(sub, sup)
    
    def add_implicit_MAPL_constants(self, typed_obj_list):
        known_constants = set(c.name for c in self._constants)
        for to in typed_obj_list:
            if to.name not in known_constants:
                self._constants.append(to)
                known_constants.add(to.name)
    
    def add_implicit_MAPL_predicate(self, pred_decl_str):
        alist = pred_decl_str.split()
        pred_name = alist[0]
        if pred_name not in self._state_variables:
            new_pred = predicates.Predicate.parse(alist)
            self._state_variables[pred_name] = new_pred

    def pddl_domain_str(self):
        elmts = self._pddl_domain_gen()
        #print list(elmts)
        return "\n".join(elmts)

    def pddl_problem_str(self):
        elmts = self._pddl_problem_gen()
        #print list(elmts)
        return "\n".join(elmts)

    def _pddl_domain_gen(self, keep_assertions=True):
        yield ";; PDDL domain '%s' (compiled from MAPL)" % self._task_name
        yield ""
        yield "(define (domain %s)" % self._domain_name
        yield self._requirements.pddl_str()
        # types
        yield "(:types"
        yield self._type_tree.to_str()
        yield ")"
        # predicates
        yield "(:predicates"
        agts = [types.TypedObject("?agt0", "agent")]
        for pred in self._state_variables.values():
            yield "  ;; predicate: %s" % pred.name
            declarations = []
#             if pred.name == "=":
#                 continue
            declarations.append(pred.pddl_str())
            kpred = pred.knowledge_pred(agts)
            declarations.append(kpred.pddl_str())
            declarations.append(kpred.pddl_str(k_axiom=True))
            for decl in declarations:
                yield "  (%s)" % decl
        yield ")"
        #constants
        yield "(:constants"
        for obj in self._constants:
            yield "  %s - %s" % (obj.name, obj.type)
        yield ")"
        # axioms
        for axiom in self._axioms:
            for line in axiom.pddl_gen():
                yield line
        # actions
        for action in self._operators:
            for line in action.pddl_gen(keep_assertions):
                yield line
        # sensors
        for sensor in self._sensors:
            for line in sensor.pddl_gen(keep_assertions):
                yield line
        yield ")"

    def _pddl_problem_gen(self):
        yield ";; PDDL problem '%s' (compiled from MAPL)" % self._domain_name
        yield "(define (problem %s)" % self._task_name
        yield "(:domain %s)" % self._domain_name
        #objects
        yield "(:objects"
        for obj in self._objects:
            yield "  %s - %s" % (obj.name, obj.type)
        yield ")"
        #init

        yield "(:init"
        for fact in self.get_state().get_facts():
            if fact.is_equality_constraint():
                continue
            factstr = fact.as_pddl_str(use_direct_k=True)
            yield "  %s" % factstr
        yield ")"
        yield "(:goal"
        yield self._goal.pddl_str()
        yield "))"

