from collections import defaultdict
from string import Template
import constants

class Graph(object):
    def __init__(self, orig=False):
        self.V = set()
        self.E = defaultdict(set)
        self.inE = defaultdict(set)
        self.labels = defaultdict(str)
        if orig:
            for v1 in orig.V:
                for v2 in orig.E[v1]:
                    self.add_edge(v1, v2, orig.labels[v1,v2])
    def __str__(self):
        def edge_str(v1,v2):
            if (v1,v2) in self.labels:
                return "%s -> %s [%s]" % (v1,v2,self.labels[v1,v2])
            else:
                return "%s -> %s" % (v1,v2)
        edges = [edge_str(v1,v2) for v1 in self.V for v2 in self.E[v1]]
        return "\n".join(sorted(edges))
    def __eq__(self, g):
        if not isinstance(g, Graph):
            return False
        if self.V != g.V:
            return False
        return all(self.E[v] == g.E[v] for v in self.V)
    @classmethod
    def parse(cls, s):
        g = Graph()
        c = 1
        for line in s.splitlines():
            if not line or not line.strip(): continue
            l, arrow, r = line.split()
            if arrow.startswith("<"):
                g.add_edge(r, l, c)
            if arrow.endswith(">"):
                g.add_edge(l ,r, c)
            c += 1
        return g
    def add_node(self, n, label=None):
        self.V.add(n)
        if label:
            self.labels[n] = label
    def del_node(self, n):
        if n not in self.V:
            return
        print "deleting node %s" % n
        self.V.remove(n)
        if n in self.labels:
            self.labels.remove(n)
        for n2 in list(self.E[n]):
            self.del_edge(n, n2)
        for n2 in list(self.inE[n]):
            self.del_edge(n2, n)
    def has_edge(self, fnode, tnode):
        return tnode in self.E[fnode]
    def add_edge(self, fnode, tnode, label=None):
        self.add_node(fnode)
        self.add_node(tnode)
        self.E[fnode].add(tnode)
        self.inE[tnode].add(fnode)
        if label:
            self.labels[fnode,tnode] = label
    def del_edge(self, fnode, tnode):
        self.E[fnode].remove(tnode)
        if (fnode,tnode) in self.labels:
            self.labels.remove((fnode,tnode))
        if not self.E[fnode]:
            self.del_node(fnode)
        self.inE[tnode].remove(fnode)
        if not self.inE[tnode]:
            self.del_node(tnode)
    def count_edges(self):
        return sum(len(self.E[v]) for v in self.V)
    def all_edges(self):
        # return edges as tuples
        return sorted((f,t) for f in self.V for t in self.E[f])
    def edge_diff(self, g):
        has, has_not = defaultdict(set), defaultdict(set)
        for v in self.V:
            if v not in g.V:
                has[v] = set(self.E[v])
            else:
                has[v].update(v2 for v2 in self.E[v] if v2 not in g.E[v])
                has_not[v].update(v2 for v2 in g.E[v] if v2 not in self.E[v])
        return has, has_not
    def transitive_closure(self):
        g = Graph(self)
        for k in self.V:
            for v1 in self.V:
                for v2 in self.V:
                    if g.has_edge(v1,k) and g.has_edge(k,v2):
                        g.add_edge(v1, v2)
        return g
    def transitive_reduction(self):
        # will only give a unique solution for DAGs
        g = Graph()
        for v1 in self.V:
            for v2 in self.V:
                if not self.has_edge(v1, v2):
                    continue
                if not any(self.has_edge(v1,k) and self.has_edge(k,v2) for k in self.V):
                    g.add_edge(v1, v2, self.labels[v1,v2])
        return g

    def to_dot(self, name="plan", ranks=[]):
        def declare_node(node_id):
            label = self.labels.get(node_id)
            if label is None: label = node_id
            return '"%s" [%s]' % (node_id, label) 
        def declare_edge(node_id1, node_id2):
            label = ""
            if (node_id1, node_id2) in self.labels:
                label = self.labels[node_id1, node_id2]
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
        

example = """
A -> B
A -> C
B -> C
B -> D
C -> D
E -> F
F -> C
"""

if __name__ == "__main__":
    g = Graph.parse(example)
    g2 = g.transitive_closure()
    g3 = g2.transitive_reduction()
    g4 = g3.transitive_closure()
    print g4.to_dot()
##     print "ORG:"
##     print g
##     print "TC:"
##     print g2
##     print "TR:"
##     print g3
##     print "Original graph had %d edges." % g.count_edges()
##     print "TC graph has %d edges." % g2.count_edges()
##     print "TR graph has %d edges." % g3.count_edges()
##     print "Org = TC? ", g == g2
##     print "Org = TR? ", g == g3
##     print "g2 = g4? ", g2 == g4
#    n = "F"
#    g.del_node(n)
#    print "\nOrig ohne %s:" % n
#    print g


