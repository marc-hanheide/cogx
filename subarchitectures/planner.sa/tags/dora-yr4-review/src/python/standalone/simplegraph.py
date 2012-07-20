from collections import defaultdict
from itertools import chain

class Graph(defaultdict):
    def __init__(self):
        defaultdict.__init__(self, set)

    def succ(self, elems):
        if not isinstance(elems, (list, set, tuple)):
            elems = [elems]
        open = set(elems)
        closed = set()
        while open:
            node = open.pop()
            closed.add(node)
            closed |= (self[node] - closed)
        return closed

    def undirected(self):
        un = Graph()
        for k, vals in self.iteritems():
            for v in vals:
                un[v].add(k)
                un[k].add(v)
        return un
    
    def reverse(self):
        rev = Graph()
        for k, vals in self.iteritems():
            for v in vals:
                rev[v].add(k)
        return rev
    
    def topological_sort(self):
        rank = defaultdict(lambda: 0)
        def update_rank(var, pred):
            r = rank[var]
            for v in self.get(var, []):
                assert v not in pred, "cycle detected"
                if rank[v] <= r:
                    rank[v] = r+1
                    update_rank(v, pred+[var])
        for v in self.iterkeys():
            if rank[v] == 0:
                update_rank(v, [])
        allvars = set(chain(self.iterkeys(), *self.itervalues()))
        return sorted(allvars, key=lambda v: rank[v])
