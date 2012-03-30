"""
Main class for storing MAPL plans. Such plans
- are partially ordered
- may contain speech acts and other higher-level actions
- store causal links and threat-prevention links between actions


"""

from string import Template
from collections import defaultdict
import copy, itertools

import networkx
import constants
import pddl
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
    def __init__(self, fnode, tnode, label=None):
        assert isinstance(fnode, PlanNode)
        assert isinstance(tnode, PlanNode)
        self.fnode = fnode
        self.tnode = tnode
        self.label = label
    def __str__(self):
        if self.label is not None:
            return self.label
        return "%s -> %s" % (self.fnode, self.tnode)


class PlanNode(object):
    def __init__(self, action, args, time, status):
        self.action = action
        self.full_args = args
        self.time = time
        self.status = status
        self.cost = 1
        self.prob = 1.0

        self.preconds = set()
        self.replanconds = set()
        self.effects = set()
        self.preconds_universal = set()
        self.replan_universal = set()

        self.ceffs = []
        self.enabled_ceffs = []

        self.original_preconds = set()
        self.original_replan = set()
        self.explanations = {}
        
        if isinstance(action, pddl.mapl.MAPLAction):
            num = len(action.agents) + len(action.params)
            self.args = args[:num]
        else:
            self.args = args

    def is_executable(self):
        if self.is_virtual():
            return False
        return self.status == ActionStatusEnum.EXECUTABLE

    def is_inprogress(self):
        return self.status == ActionStatusEnum.IN_PROGRESS

    def is_virtual(self):
        return self.action.name.startswith("select-") or self.action.name.startswith("__")

    def __str__(self):
        #return "%s(%s)" % (self.action, self.time)
        def tostr(arg):
            return a.name if hasattr(a,"name") else str(a)
        args = [tostr(a) for a in self.args]
        return "%d: " % self.time + " ".join([self.action.name]+args)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and self.action.name == other.action.name and all(map(lambda a,b: a == b, self.full_args, other.full_args))
    
    def copy(self):
        return self.__class__(self.action, list(self.full_args), self.time, self.status)

class DummyAction(object):
    def __init__(self, name):
        self.name = name
        self.replan = None
    def __str__(self):
        return str(self.name)

class GoalAction(DummyAction):
    def __init__(self, goal):
        self.name = "goal"
        self.replan = None
        self.precondition = goal
    def instantiate(self, x, y):
        pass
    def uninstantiate(self):
        pass
    def __str__(self):
        return str(self.name)

class DummyNode(PlanNode):
    def __init__(self, name, args, time, status):
        action = DummyAction(name)
        PlanNode.__init__(self, action, args, time, status)
        
        self.cost = 0
        self.prob = 1.0
        
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

    def get_total_costs(self):
        return sum(n.cost for n in self.nodes_iter())
            
    def get_remaining_costs(self):
        return sum(n.cost for n in self.nodes_iter() if n.status == ActionStatusEnum.EXECUTABLE)

    def add_link(self, n1, n2, svar, val, conflict=False, **add_args ):
        if conflict:
            type = "prevent_threat"
        else:
            type = "depends"
        self.add_edge(n1, n2,  svar=svar, val=val, type=type, **add_args)

    def replace_link(self, n1, n2, svar, val, _type):
        if n1 in self and n2 in self[n1]:
            for key, e in self[n1][n2].items():
                if e['svar'] == svar:
                    del self[n1][n2][key]
        self.add_edge(n1, n2, svar=svar, val=val, type=_type)
    
    def topological_sort(self):
        return networkx.topological_sort(self)

    def executable(self):
        result = set()
        for n in self.nodes_iter():
            if n.status != ActionStatusEnum.EXECUTABLE:
                continue
            if all(pred.status == ActionStatusEnum.EXECUTED for pred in self.predecessors(n)):
                result.add(n)
                
        return result

    def find_loop(self):
        stack = []
        closed = set()
        def dfs(node):
            if node in stack:
                print "Found loop:"
                for n in itertools.dropwhile(lambda x: x != node, stack):
                    print "  ", n
                print "  ", node
                return True
            stack.append(node)
            for succ in self.successors_iter(node):
                if succ not in closed:
                    if dfs(succ):
                        return True
                    closed.add(succ)
            stack.pop(-1)
            return False
        return dfs(self.init_node)
                
                
    def compute_depths(self):
        visited = set()
        def visit(n):
            if n not in visited:
                visited.add(n)
                predecessors = self.predecessors(n)
                for pred in predecessors:
                    visit(pred)
                    
                if predecessors:
                    self.node[n]['depth'] = max(self.node[pred]['depth'] for pred in predecessors) + 1
                else:
                    self.node[n]['depth'] = 0
        for n in self.nodes_iter():
            visit(n)

    def incoming_links(self, n):
        if n not in self:
            return
        for pred in self.predecessors_iter(n):
            for e in self[pred][n].itervalues():
                yield pred, e.get("svar",None), e.get("val",None), e.get("type",None)

    def outgoing_links(self, n):
        if n not in self:
            return
        for succ in self.successors_iter(n):
            for e in self[n][succ].itervalues():
                yield succ, e.get("svar",None), e.get("val",None), e.get("type",None)
                
    def predecessors_iter(self, node, link_type=None):
        if link_type and not isinstance(link_type, (set, list, tuple)):
            link_type = [link_type]
        for p in networkx.MultiDiGraph.predecessors_iter(self, node):
            if not link_type or any(e['type'] in link_type for e in self[p][node].itervalues()):
                yield p
                
    def predecessors(self, node, link_type=None):
        if not link_type:
            return networkx.MultiDiGraph.predecessors(self, node)
        return [p for p in self.predecessors_iter(node, link_type)]

    def successors_iter(self, node, link_type=None):
        if link_type and not isinstance(link_type, (set, list, tuple)):
            link_type = [link_type]
        for s in networkx.MultiDiGraph.successors_iter(self, node):
            if not link_type or any(e['type'] in link_type for e in self[node][s].itervalues()):
                yield s
                
    def successors(self, node, link_type=None):
        if not link_type:
            return networkx.MultiDiGraph.successors(self, node)
        return [n for n in self.successors_iter(node, link_type)]
    
    def pred_closure(self, node, link_type=None):
        open = set([node])
        closed = set([self.init_node])
        result = set()
        while open:
            node = open.pop()
            closed.add(node)
            pred = set(self.predecessors(node, link_type))
            result |= pred
            open |= (pred - closed)
        return result

    def succ_closure(self, node, link_type=None):
        open = set([node])
        closed = set()
        result = set()
        while open:
            node = open.pop()
            closed.add(node)
            succ = set(self.successors(node, link_type))
            result |= succ
            open |= (succ - closed)
        return result

    def __str__(self):
        nodes = self.topological_sort()
        return "\n".join(map(str, nodes))

    def to_dot(self, name="plan", ranks=[], node_deco=None, edge_deco=None):
        from pygraphviz import AGraph
        def declare_rank(same_rank_list):
            same_rank_list = ['"%s"' % r for r in same_rank_list]
            return '{rank=same; %s}' % " ".join(same_rank_list)

        cyclic = self.find_loop()
        if not cyclic:
            self.compute_depths()
        ranks = defaultdict(list)
        G = AGraph(directed=True, strict=False)
        for n, data in self.nodes_iter(data=True):
            # if n == self.init_node:
            #     continue
            if not cyclic:
                ranks[data['depth']].append(n)
            attrs = {}
            attrs["style"] = "filled"
            if n.status == ActionStatusEnum.EXECUTED:
                attrs["fillcolor"] = "grey"
            elif n.status == ActionStatusEnum.FAILED:
                attrs["fillcolor"] = "tomato1"
            else:
                attrs["fillcolor"] = "white"
                
            if n.action.replan:
                attrs["shape"] = "box"
                attrs["style"] += ", rounded, dashed"
            elif isinstance(n.action, pddl.mapl.MAPLAction) and n.action.sensors:
                attrs["shape"] = "box"
                attrs["style"] += ", rounded"

            if node_deco:
                d = node_deco(n)
                if d:
                    attrs.update(d)
            
            if not attrs.get('ignore', False):
                G.add_node(n, **attrs)


        for n1,n2, data in self.edges_iter(data=True):
            # if n1 == self.init_node:
            #     continue
            if n1 not in G or n2 not in G:
                continue
            attrs = {}
            if data['type'] == 'prevent_threat':
                attrs["style"] = "dashed"
                attrs["color"] = "darkgreen"
                attrs["label"] = str(data['svar'])
                # continue
            elif data['type'] == 'unexpected':
                attrs["color"] = "red"
                attrs["label"] = "%s = %s" % (str(data['svar']), data['val'].name)
            elif data.get('svar', None) is not None and data.get('val', None) is not None:
                attrs["label"] = "%s = %s" % (str(data['svar']), data['val'].name)
            
            if edge_deco:
                d = edge_deco(n1, n2, data)
                if d:
                    attrs.update(d)
                
            if not attrs.get('ignore', False):
                G.add_edge(n1, n2, **attrs)

        for rank, nodes in ranks.iteritems():
            G.add_subgraph([n for n in nodes if n in G], rank='same', label="rank %d" % rank)

        return G
