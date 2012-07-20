class UnionFindNode(object):
    def __init__(self, data):
        self.parent = self
        self.depth = 0
        self.data = data

        
class UnionFind(dict):
    def __init__(self, data):
        self.update((d,UnionFindNode(d)) for d in data)

    def __getitem__(self, item):
        if item not in self:
            n = UnionFindNode(item)
            self[item] = n
            return n
        return dict.__getitem__(self, item)

    def to_sets(self):
        rootsets = {}
        for d, n in self.iteritems():
            n = self._find(d)
            if n not in rootsets:
                rootsets[n] = set((n.data, d))
            else:
                rootsets[n].add(d)
        return rootsets.values()

    def union(self, d1, d2):
        n1 = self._find(d1)
        n2 = self._find(d2)
        if n1 == n2:
            return
        
        if n1.depth < n2.depth:
            n1.parent = n2
            n1.data = d2
            n2.data = d1
            self[d1] = n2
            self[d2] = n1
        elif n1.depth > n2.depth:
            n2.parent = n1
        else:
            n2.parent = n1
            n1.depth += 1

    def find(self, data):
        return self._find(data).data
    
    def _find(self, data):
        path = []
        n = self[data]
        while n.parent != n:
            path.append(n)
            n = n.parent
        for c in path:
            c.parent = n
        return n
            
