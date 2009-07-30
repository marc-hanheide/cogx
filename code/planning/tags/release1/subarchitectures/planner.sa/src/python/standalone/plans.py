"""
Main class for storing MAPL plans. Such plans
- are partially ordered
- may contain speech acts and other higher-level actions
- store causal links and threat-prevention links between actions


"""

from string import Template

import graph
import constants

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
    def __init__(self, action, time):
        self.action = action
        self.time = time
    def __str__(self):
        #return "%s(%s)" % (self.action, self.time)
        return "%s" % self.action
        
    

class MAPLPlan(graph.DAG):
    def __init__(self, init_state=None, goal_condition=None):
        graph.DAG.__init__(self)
        self.init_node = self.create_init_node(init_state)
        self.goal_node = self.create_goal_node(goal_condition)
    def add_link(self, node_or_link, node2=None):
        if node2:
            # create link from the two PlanNodes given.
            # types are checked in the OrderingConstraint constructor
            link = OrderingConstraint(node_or_link, node2)
        else:
            link = node_or_link
        self.add_edge(link.fnode, link.tnode, link)
    def create_init_node(self, astate):
        return PlanNode("init", 0)
    def create_goal_node(self, astate):
        return PlanNode("goal", 9999)
    def _edge_str(self, v1, v2):
        label = self.labels.get((v1,v2))
        assert isinstance(label, OrderingConstraint)
        return str(label)
    def to_dot(self, name="plan", ranks=[]):
        def declare_node(node_id):
            label = self.labels.get(node_id)
            if label is None: label = node_id
            return '"%s" [%s]' % (node_id, label) 
        def declare_edge(node_id1, node_id2):
            label = ""
#             if (node_id1, node_id2) in self.labels:
#                 label = self.labels[node_id1, node_id2]
            return '"%s" -> "%s" [%s]' % (node_id1, node_id2, label)
        def declare_rank(same_rank_list):
            same_rank_list = ['"%s"' % r for r in same_rank_list]
            return '{rank=same; %s}' % " ".join(same_rank_list)
        node_decl = "\n".join(declare_node(n) for n in sorted(self.V))
        edge_decl = "\n".join(declare_edge(n1, n2) for (n1, n2) in self.all_edges())
        ranks = "\n".join(declare_rank(r) for r in ranks)
        setup = constants.DOT_SETUP_TMPL
        dot_str = Template(constants.DOT_TEMPLATE).safe_substitute(locals())
        return dot_str
