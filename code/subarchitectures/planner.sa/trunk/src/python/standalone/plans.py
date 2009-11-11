"""
Main class for storing MAPL plans. Such plans
- are partially ordered
- may contain speech acts and other higher-level actions
- store causal links and threat-prevention links between actions


"""

from string import Template
import copy

import networkx
import graph
import constants
from utils import Enum

ActionStatusEnum = Enum("EXECUTABLE", "IN_PROGRESS", "EXECUTED", "FAILED")    

class OrderingConstraint(object):
    """
    An OrderingConstraint is a link between two PlanNodes in a MAPLPlan.
    This is the super class for
    - CausalLink
    - ThreatPreventionLink
    and can be used, e.g., for links in the transitive closure of plans using
    any of the above specialised links.
    """
    def __init__(self, fnode, tnode):
        assert isinstance(fnode, PlanNode)
        assert isinstance(tnode, PlanNode)
        self.fnode = fnode
        self.tnode = tnode
    def __str__(self):
        return "%s -> %s" % (self.fnode, self.tnode)


class PlanNode(object):
    def __init__(self, action, args, time, status):
        self.action = action
        self.full_args = args
        self.time = time
        self.status = status

        self.preconds = set()
        self.replanconds = set()
        self.effects = set()
        self.preconds_universal = set()
        self.replan_universal = set()
        
        if not isinstance(action, DummyAction):
            num = len(action.agents) + len(action.args)
            self.args = args[:num]
        else:
            self.args = args

    def is_executable(self):
        return self.status == ActionStatusEnum.EXECUTABLE

    def is_inprogress(self):
        return self.status == ActionStatusEnum.IN_PROGRESS

    def __str__(self):
        #return "%s(%s)" % (self.action, self.time)
        def tostr(arg):
            return a.name if hasattr(a,"name") else str(a)
        args = [tostr(a) for a in self.args]
        return " ".join([self.action.name]+args)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.action.name == other.action.name and all(map(lambda a,b: a == b, self.full_args, other.full_args))
    
    def copy(self):
        return self.__class__(self.action, list(self.full_args), self.time, self.status)

class DummyAction(object):
    def __init__(self, name):
        self.name = name
    def __str__(self):
        return str(self.name)

class GoalAction(DummyAction):
    def __init__(self, goal):
        self.name = "goal"
        self.replan = None
        self.precondition = goal
    def instantiate(self, x):
        pass
    def uninstantiate(self):
        pass
    def __str__(self):
        return str(self.name)

class DummyNode(PlanNode):
    def __init__(self, name, args, time, status):
        action = DummyAction(name)
        PlanNode.__init__(self, action, args, time, status)
    def __str__(self):
        return str(self.action)
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.action == other.action

class MAPLPlan(networkx.MultiDiGraph):
    def __init__(self, init_state=None, goal_condition=None):
        networkx.MultiDiGraph.__init__(self)

        self.init_node = self.create_init_node(init_state)
        self.goal_node = self.create_goal_node(goal_condition)
        self.add_node(self.init_node)
        self.add_node(self.goal_node)
        self.execution_position = 0

    def create_init_node(self, astate):
        ## TODO: astate is still unused
        return DummyNode("init", [], 0, ActionStatusEnum.EXECUTED)
    def create_goal_node(self, astate):
        ## TODO: astate is still unused
        return DummyNode("goal", [], 9999, ActionStatusEnum.EXECUTABLE)

    def add_link(self, n1, n2, svar, val, conflict=False):
        if conflict:
            type = "prevent_threat"
        else:
            type = "depends"
        self.add_edge(n1, n2,  svar=svar, val=val, type=type)
    
    def topological_sort(self):
        return networkx.topological_sort(self)

    def pred_closure(self, node):
        open = set([node])
        closed = set([self.init_node])
        result = set([node])
        while open:
            node = open.pop()
            closed.add(node)
            pred = self.predecessors(node)
            result |= pred
            open |= (pred - closed)
        return result

    def succ_closure(self, node):
        open = set([node])
        closed = set()
        result = set([node])
        while open:
            node = open.pop()
            closed.add(node)
            pred = self.successors(node)
            result |= pred
            open |= (pred - closed)
        return result
    
# class MAPLPlan(graph.DAG):
#     def __init__(self, init_state=None, goal_condition=None):
#         graph.DAG.__init__(self)
#         self.init_node = self.create_init_node(init_state)
#         self.goal_node = self.create_goal_node(goal_condition)
#         self.add_node(self.init_node)
#         self.add_node(self.goal_node)
#         self.execution_position = 0
        
#     def copy(self):
#         p = MAPLPlan()
#         n2n = {}
#         for n in self.V:
#             new_n = n2n.get(n, n.copy())
#             for n2 in self.E[n]:
#                 new_n2 = n2n.get(n2, n2.copy())
#                 p.add_edge(new_n, new_n2, self.labels[n,n2])
#                 for old, new in ((n,new_n), (n2, new_n2)):
#                     if old == self.init_node:
#                         p.init_node = new
#                     if old == self.goal_node:
#                         p.goal_node = old
#         return p
#     def add_link(self, node_or_link, node2=None):
#         if node2:
#             # create link from the two PlanNodes given.
#             # types are checked in the OrderingConstraint constructor
#             link = OrderingConstraint(node_or_link, node2)
#         else:
#             link = node_or_link
#         self.add_edge(link.fnode, link.tnode, link)
#     def create_init_node(self, astate):
#         ## TODO: astate is still unused
#         return DummyNode("init", [], 0, ActionStatusEnum.EXECUTED)
#     def create_goal_node(self, astate):
#         ## TODO: astate is still unused
#         return DummyNode("goal", [], 9999, ActionStatusEnum.EXECUTABLE)
#     def _edge_str(self, v1, v2):
#         label = self.labels.get((v1,v2))
#         assert isinstance(label, OrderingConstraint)
#         return str(label)
#     def all_actions(self):
#         return self.V
#     def to_dot(self, name="plan", ranks=[]):
#         def declare_node(node_id):
#             label = self.labels.get(node_id)
#             if label is None: label = node_id
#             params = ""
#             if node_id.status == ActionStatusEnum.EXECUTED:
#                 params += "style=filled, fillcolor=grey, "
#             return '"%s" [%s "%s"]' % (node_id, params, label) 
#         def declare_edge(node_id1, node_id2):
#             label = ""
# #             if (node_id1, node_id2) in self.labels:
# #                 label = self.labels[node_id1, node_id2]
#             return '"%s" -> "%s" [%s]' % (node_id1, node_id2, label)
#         def declare_rank(same_rank_list):
#             same_rank_list = ['"%s"' % r for r in same_rank_list]
#             return '{rank=same; %s}' % " ".join(same_rank_list)
#         node_decl = "\n".join(declare_node(n) for n in sorted(self.V))
#         edge_decl = "\n".join(declare_edge(n1, n2) for (n1, n2) in self.all_edges())
#         ranks = "\n".join(declare_rank(r) for r in ranks)
#         setup = constants.DOT_SETUP_TMPL
#         dot_str = Template(constants.DOT_TEMPLATE).safe_substitute(locals())
#         return dot_str
