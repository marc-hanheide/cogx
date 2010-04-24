"""
Base class for undirected graphs.

The Graph class allows any hashable object as a node 
and can associate key/value attribute pairs with each undirected edge.

Self-loops are allowed but multiple edges are not (see MultiGraph).

For directed graphs see DiGraph and MultiDiGraph.

"""
__author__ = """\n""".join(['Aric Hagberg (hagberg@lanl.gov)',
                            'Pieter Swart (swart@lanl.gov)',
                            'Dan Schult(dschult@colgate.edu)'])
#    Copyright (C) 2004-2009 by 
#    Aric Hagberg <hagberg@lanl.gov>
#    Dan Schult <dschult@colgate.edu>
#    Pieter Swart <swart@lanl.gov>
#    Distributed under the terms of the GNU Lesser General Public License
#    http://www.gnu.org/copyleft/lesser.html
#
__docformat__ = "restructuredtext en"

from networkx.exception import NetworkXException, NetworkXError
import networkx.convert as convert
from copy import deepcopy

class Graph(object):
    """
    Base class for undirected graphs.

    A Graph stores nodes and edges with optional data, or attributes.

    Graphs hold undirected edges.  Self loops are allowed but multiple
    (parallel) edges are not.

    Nodes can be arbitrary (hashable) Python objects with optional
    key/value attributes.

    Edges are represented as links between nodes with optional
    key/value attributes.

    Parameters
    ----------
    data : input graph
        Data to initialize graph.  If data=None (default) an empty
        graph is created.  The data can be an edge list, or any
        NetworkX graph object.  If the corresponding optional Python
        packages are installed the data can also be a NumPy matrix
        or 2d ndarray, a SciPy sparse matrix, or a PyGraphviz graph.
    name : string, optional (default='')
        An optional name for the graph.
    attr : keyword arguments, optional (default= no attributes)
        Attributes to add to graph as key=value pairs.

    See Also
    --------
    DiGraph
    MultiGraph
    MultiDiGraph
    
    Examples
    --------
    Create an empty graph structure (a "null graph") with no nodes and 
    no edges.

    >>> G = nx.Graph()

    G can be grown in several ways.

    **Nodes:**
  
    Add one node at a time:

    >>> G.add_node(1)

    Add the nodes from any container (a list, dict, set or 
    even the lines from a file or the nodes from another graph).

    >>> G.add_nodes_from([2,3])
    >>> G.add_nodes_from(range(100,110))
    >>> H=nx.path_graph(10)
    >>> G.add_nodes_from(H)

    In addition to strings and integers any hashable Python object
    (except None) can represent a node, e.g. a customized node object,
    or even another Graph.

    >>> G.add_node(H)

    **Edges:**
    
    G can also be grown by adding edges.

    Add one edge,

    >>> G.add_edge(1, 2)

    a list of edges, 

    >>> G.add_edges_from([(1,2),(1,3)])

    or a collection of edges,
    
    >>> G.add_edges_from(H.edges())

    If some edges connect nodes not yet in the graph, the nodes 
    are added automatically.  There are no errors when adding 
    nodes or edges that already exist. 

    **Attributes:**
    
    Each graph, node, and edge can hold key/value attribute pairs
    in an associated attribute dictionary (the keys must be hashable).
    By default these are empty, but can be added or changed using
    add_edge, add_node or direct manipulation of the attribute 
    dictionaries named graph, node and edge respectively.

    >>> G = nx.Graph(day="Friday")
    >>> G.graph
    {'day': 'Friday'}

    Add node attributes using add_node(), add_nodes_from() or G.node

    >>> G.add_node(1, time='5pm')
    >>> G.add_nodes_from([3], time='2pm')
    >>> G.node[1]
    {'time': '5pm'}
    >>> G.node[1]['room'] = 714
    >>> G.nodes(data=True)
    [(1, {'room': 714, 'time': '5pm'}), (3, {'time': '2pm'})]

    Warning: adding a node to G.node does not add it to the graph.

    Add edge attributes using add_edge(), add_edges_from(), subscript
    notation, or G.edge.

    >>> G.add_edge(1, 2, weight=4.7 )
    >>> G.add_edges_from([(3,4),(4,5)], color='red')
    >>> G.add_edges_from([(1,2,{'color':'blue'}), (2,3,{'weight':8})])
    >>> G[1][2]['weight'] = 4.7
    >>> G.edge[1][2]['weight'] = 4

    **Shortcuts:**

    Many common graph features allow python syntax to speed reporting.

    >>> 1 in G     # check if node in graph
    True
    >>> print [n for n in G if n<3]   # iterate through nodes
    [1, 2]
    >>> print len(G)  # number of nodes in graph
    5
    >>> print G[1] # adjacency dict keyed by neighbor to edge attributes
    ...            # Note: you should not change this dict manually!
    {2: {'color': 'blue', 'weight': 4}}

    The fastest way to traverse all edges of a graph is via 
    adjacency_iter(), but the edges() method is often more convenient.

    >>> for n,nbrsdict in G.adjacency_iter(): 
    ...     for nbr,eattr in nbrsdict.iteritems():
    ...        if 'weight' in eattr:
    ...            print (n,nbr,eattr['weight'])
    (1, 2, 4)
    (2, 1, 4)
    (2, 3, 8)
    (3, 2, 8)
    >>> print [ (u,v,edata['weight']) for u,v,edata in G.edges(data=True) if 'weight' in edata ]
    [(1, 2, 4), (2, 3, 8)]

    **Reporting:**
    
    Simple graph information is obtained using methods.
    Iterator versions of many reporting methods exist for efficiency.
    Methods exist for reporting nodes(), edges(), neighbors() and degree()
    as well as the number of nodes and edges.
    
    For details on these and other miscellaneous methods, see below.
    """
    def __init__(self, data=None, name='', **attr):
        """Initialize a graph with edges, name, graph attributes.

        Parameters
        ----------
        data : input graph
            Data to initialize graph.  If data=None (default) an empty
            graph is created.  The data can be an edge list, or any
            NetworkX graph object.  If the corresponding optional Python
            packages are installed the data can also be a NumPy matrix
            or 2d ndarray, a SciPy sparse matrix, or a PyGraphviz graph.
        name : string, optional (default='')
            An optional name for the graph.
        attr : keyword arguments, optional (default= no attributes)
            Attributes to add to graph as key=value pairs.

        See Also
        --------
        convert
            
        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G = nx.Graph(name='my graph')
        >>> e = [(1,2),(2,3),(3,4)] # list of edges
        >>> G = nx.Graph(e)

        Arbitrary graph attribute pairs (key=value) may be assigned

        >>> G=nx.Graph(e, day="Friday")
        >>> G.graph
        {'day': 'Friday'}

        """
        self.graph = {}   # dictionary for graph attributes
        self.node = {}    # empty node dict (created before convert)
        self.adj = {}     # empty adjacency dict
        # attempt to load graph with data
        if data is not None:
            convert.from_whatever(data,create_using=self)
        # load graph attributes (must be after convert)
        self.graph.update(attr)
        self.name = name
        self.edge = self.adj

    def __str__(self):
        """Return the graph name.

        Returns
        -------
        name : string
            The name of the graph.

        Examples
        --------
        >>> G = nx.path_graph(4)        
        >>> print G
        path_graph(4)
        """
        return self.name

    def __iter__(self):
        """Iterate over the nodes. Use the expression 'for n in G'.

        Returns
        -------
        niter : iterator
            An iterator over all nodes in the graph.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> for n in G:
        ...     print n,
        0 1 2 3
        """
        return self.adj.iterkeys()

    def __contains__(self,n):
        """Return True if n is a node, False otherwise. Use the expression
        'n in G'.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> print 1 in G
        True
        """
        try:
            return n in self.adj
        except TypeError:
            return False
        
    def __len__(self):
        """Return the number of nodes. Use the expression 'len(G)'.

        Returns
        -------
        nnodes : int
            The number of nodes in the graph.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> print len(G)
        4

        """
        return len(self.adj)

    def __getitem__(self, n):
        """Return a dict of neighbors of node n.  Use the expression 'G[n]'.

        Parameters
        ----------
        n : node
           A node in the graph.

        Returns
        -------
        adj_dict : dictionary
           The adjacency dictionary for nodes connected to n.

        Notes
        -----
        G[n] is similar to G.neighbors(n) but the internal data dictionary
        is returned instead of a list.

        Assigning G[n] will corrupt the internal graph data structure.
        Use G[n] for reading data only.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> print G[0]
        {1: {}}
        """
        return self.adj[n]
    

    def add_node(self, n, attr_dict=None, **attr):
        """Add a single node n and update node attributes.

        Parameters
        ----------
        n : node
            A node can be any hashable Python object except None.
        attr_dict : dictionary, optional (default= no attributes)
            Dictionary of node attributes.  Key/value pairs will
            update existing data associated with the node.
        attr : keyword arguments, optional
            Set or change attributes using key=value.

        See Also
        --------
        add_nodes_from

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_node(1)
        >>> G.add_node('Hello')
        >>> K3 = nx.Graph([(0,1),(1,2),(2,0)])
        >>> G.add_node(K3)
        >>> G.number_of_nodes()
        3

        Use keywords set/change node attributes:

        >>> G.add_node(1,size=10)
        >>> G.add_node(3,weight=0.4,UTM=('13S',382871,3972649))

        Notes
        -----
        A hashable object is one that can be used as a key in a Python
        dictionary. This includes strings, numbers, tuples of strings
        and numbers, etc.

        On many platforms hashable items also include mutables such as
        NetworkX Graphs, though one should be careful that the hash
        doesn't change on mutables.
        """
        # set up attribute dict
        if attr_dict is None:
            attr_dict=attr
        else:
            try:
                attr_dict.update(attr)
            except AttributeError:
                raise NetworkXError(\
                    "The attr_dict argument must be a dictionary.")
        if n not in self.adj:
            self.adj[n] = {}
            self.node[n] = attr
        else: # update attr even if node already exists            
            self.node[n].update(attr_dict)


    def add_nodes_from(self, nodes, **attr):
        """Add multiple nodes.

        Parameters
        ----------
        nodes : iterable container
            A container of nodes (list, dict, set, etc.).  The
            container will be iterated through once.
        attr : keyword arguments, optional (default= no attributes)
            Update attributes for all nodes in nodes.

        See Also
        --------
        add_node

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_nodes_from('Hello')
        >>> K3 = nx.Graph([(0,1),(1,2),(2,0)])
        >>> G.add_nodes_from(K3)
        >>> sorted(G.nodes())
        [0, 1, 2, 'H', 'e', 'l', 'o']

        Use keywords to update specific node attributes for every node.

        >>> G.add_nodes_from([1,2], size=10)
        >>> G.add_nodes_from([3,4], weight=0.4)

        """
        for n in nodes:
            if n not in self.adj:
                self.adj[n] = {}
                self.node[n] = attr.copy()
            else:
                self.node[n].update(attr)

    def remove_node(self,n):
        """Remove node n.

        Removes the node n and all adjacent edges.
        Attempting to remove a non-existent node will raise an exception.

        Parameters
        ----------
        n : node
           A node in the graph

        Raises
        -------
        NetworkXError
           If n is not in the graph.

        See Also
        --------
        remove_nodes_from

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> G.edges()
        [(0, 1), (1, 2)]
        >>> G.remove_node(1)
        >>> G.edges()
        []

        """
        adj = self.adj
        try:
            nbrs = adj[n].keys() # keys handles self-loops (allow mutation later)
            del self.node[n]
        except KeyError: # NetworkXError if n not in self
            raise NetworkXError("The node %s is not in the graph."%(n,))
        for u in nbrs:  
            del adj[u][n]   # remove all edges n-u in graph
        del adj[n]          # now remove node


    def remove_nodes_from(self, nodes):
        """Remove multiple nodes.

        Parameters
        ----------
        nodes : iterable container
            A container of nodes (list, dict, set, etc.).  If a node
            in the container is not in the graph it is silently
            ignored.

        See Also
        --------
        remove_node

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> e = G.nodes()
        >>> e
        [0, 1, 2]
        >>> G.remove_nodes_from(e)
        >>> G.nodes()
        []

        """
        adj = self.adj
        for n in nodes:
            try: 
                del self.node[n]
                for u in adj[n].keys():   # keys() handles self-loops 
                    del adj[u][n]         #(allows mutation of dict in loop)
                del adj[n]
            except KeyError:
                pass


    def nodes_iter(self, data=False):
        """Return an iterator over the nodes.

        Parameters
        ----------
        data : boolean, optional (default=False)
               If False the iterator returns nodes.  If True
               return a two-tuple of node and node data dictionary

        Returns
        -------
        niter : iterator
            An iterator over nodes.  If data=True the iterator gives 
            two-tuples containing (node, node data, dictionary) 

        Notes
        -----
        If the node data is not required it is simpler and equivalent 
        to use the expression 'for n in G'.

        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> for n in G:
        ...     print n,
        0 1 2


        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> for n in G.nodes_iter():
        ...     print n,
        0 1 2
        >>> for n,d in G.nodes_iter(data=True):
        ...     print d,
        {} {} {}

        """
        if data:
            return self.node.iteritems()
        return self.adj.iterkeys()

    def nodes(self, data=False):
        """Return a list of the nodes in the graph.

        Parameters
        ----------
        data : boolean, optional (default=False) 
               If False return a list of nodes.  If True return a
               two-tuple of node and node data dictionary

        Returns
        -------
        nlist : list
            A list of nodes.  If data=True a list of two-tuples containing
            (node, node data dictionary). 

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> print G.nodes()
        [0, 1, 2]
        >>> G.add_node(1, time='5pm')
        >>> print G.nodes(data=True)
        [(0, {}), (1, {'time': '5pm'}), (2, {})]
        """
        return list(self.nodes_iter(data=data))

    def number_of_nodes(self):
        """Return the number of nodes in the graph.

        Returns
        -------
        nnodes : int
            The number of nodes in the graph.

        See Also
        --------
        order, __len__  which are identical 

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> print len(G)
        3

        """
        return len(self.adj)

    def order(self):
        """Return the number of nodes in the graph.

        Returns
        -------
        nnodes : int
            The number of nodes in the graph.

        See Also
        --------
        number_of_nodes, __len__  which are identical 

        """
        return len(self.adj)

    def has_node(self, n):
        """Return True if the graph contains the node n.

        Parameters
        ----------
        n : node

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2])
        >>> print G.has_node(0)
        True

        It is more readable and simpler to use

        >>> 0 in G
        True
        
        """
        try:
            return n in self.adj
        except TypeError:
            return False

    def add_edge(self, u, v, attr_dict=None, **attr):  
        """Add an edge between u and v.

        The nodes u and v will be automatically added if they are 
        not already in the graph.  

        Edge attributes can be specified with keywords or by providing
        a dictionary with key/value pairs.  See examples below.

        Parameters
        ----------
        u,v : nodes
            Nodes can be, for example, strings or numbers. 
            Nodes must be hashable (and not None) Python objects.
        attr_dict : dictionary, optional (default= no attributes)
            Dictionary of edge attributes.  Key/value pairs will
            update existing data associated with the edge.
        attr : keyword arguments, optional
            Edge data (or labels or objects) can be assigned using
            keyword arguments.   

        See Also
        --------
        add_edges_from : add a collection of edges

        Notes 
        -----
        Adding an edge that already exists updates the edge data.

        NetworkX algorithms designed for weighted graphs use as
        the edge weight a numerical value assigned to the keyword
        'weight'.

        Examples
        --------
        The following all add the edge e=(1,2) to graph G:
        
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> e = (1,2)
        >>> G.add_edge(1, 2)           # explicit two-node form
        >>> G.add_edge(*e)             # single edge as tuple of two nodes
        >>> G.add_edges_from( [(1,2)] ) # add edges from iterable container

        Associate data to edges using keywords:

        >>> G.add_edge(1, 2, weight=3)
        >>> G.add_edge(1, 3, weight=7, capacity=15, length=342.7)
        """
        # set up attribute dictionary
        if attr_dict is None:
            attr_dict=attr
        else:
            try:
                attr_dict.update(attr)
            except AttributeError:
                raise NetworkXError(\
                    "The attr_dict argument must be a dictionary.")
        # add nodes            
        if u not in self.adj: 
            self.adj[u] = {}
            self.node[u] = {}
        if v not in self.adj: 
            self.adj[v] = {}
            self.node[v] = {}
        # add the edge
        datadict=self.adj[u].get(v,{})
        datadict.update(attr_dict)
        self.adj[u][v] = datadict
        self.adj[v][u] = datadict


    def add_edges_from(self, ebunch, attr_dict=None, **attr):  
        """Add all the edges in ebunch.

        Parameters
        ----------
        ebunch : container of edges
            Each edge given in the container will be added to the
            graph. The edges must be given as as 2-tuples (u,v) or
            3-tuples (u,v,d) where d is a dictionary containing edge
            data.
        attr_dict : dictionary, optional (default= no attributes)
            Dictionary of edge attributes.  Key/value pairs will
            update existing data associated with each edge.
        attr : keyword arguments, optional
            Edge data (or labels or objects) can be assigned using
            keyword arguments.   


        See Also
        --------
        add_edge : add a single edge
        add_weighted_edges_from : convenient way to add weighted edges

        Notes
        -----
        Adding the same edge twice has no effect but any edge data
        will be updated when each duplicate edge is added.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edges_from([(0,1),(1,2)]) # using a list of edge tuples
        >>> e = zip(range(0,3),range(1,4))
        >>> G.add_edges_from(e) # Add the path graph 0-1-2-3
  
        Associate data to edges

        >>> G.add_edges_from([(1,2),(2,3)], weight=3)
        >>> G.add_edges_from([(3,4),(1,4)], label='WN2898')
        """
        # set up attribute dict
        if attr_dict is None:
            attr_dict=attr
        else:
            try:
                attr_dict.update(attr)
            except AttributeError:
                raise NetworkXError(\
                    "The attr_dict argument must be a dictionary.")
        # process ebunch
        for e in ebunch:
            ne=len(e)
            if ne==3:
                u,v,dd = e
            elif ne==2:
                u,v = e  
                dd = {}
            else: 
                raise NetworkXError(\
                    "Edge tuple %s must be a 2-tuple or 3-tuple."%(e,))
            if u not in self.adj: 
                self.adj[u] = {}
                self.node[u] = {}
            if v not in self.adj: 
                self.adj[v] = {}
                self.node[v] = {}
            datadict=self.adj[u].get(v,{})
            datadict.update(attr_dict) 
            datadict.update(dd)
            self.adj[u][v] = datadict
            self.adj[v][u] = datadict


    def add_weighted_edges_from(self, ebunch, **attr):  
        """Add all the edges in ebunch as weighted edges with specified
        weights.

        Parameters
        ----------
        ebunch : container of edges
            Each edge given in the list or container will be added
            to the graph. The edges must be given as 3-tuples (u,v,w)
            where w is a number.
        attr : keyword arguments, optional (default= no attributes)
            Edge attributes to add/update for all edges.

        See Also
        --------
        add_edge : add a single edge
        add_edges_from : add multiple edges

        Notes
        -----
        Adding the same edge twice has no effect but any edge data
        will be updated when each duplicate edge is added.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_weighted_edges_from([(0,1,3.0),(1,2,7.5)]) 
        """
        self.add_edges_from(((u,v,{'weight':d}) for u,v,d in ebunch),**attr)

    def remove_edge(self, u, v): 
        """Remove the edge between u and v.

        Parameters
        ----------
        u,v: nodes 
            Remove the edge between nodes u and v.

        Raises
        ------
        NetworkXError
            If there is not an edge between u and v.

        See Also
        --------
        remove_edges_from : remove a collection of edges

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.remove_edge(0,1)
        >>> e = (1,2)
        >>> G.remove_edge(*e) # unpacks e from an edge tuple
        >>> e = (2,3,{'weight':7}) # an edge with attribute data
        >>> G.remove_edge(*e[:2]) # select first part of edge tuple
        """
        try:
            del self.adj[u][v]   
            if u != v:  # self-loop needs only one entry removed
                del self.adj[v][u]   
        except KeyError: 
            raise NetworkXError("The edge %s-%s not in graph"%(u,v))



    def remove_edges_from(self, ebunch): 
        """Remove all edges specified in ebunch.

        Parameters
        ----------
        ebunch: list or container of edge tuples
            Each edge given in the list or container will be removed 
            from the graph. The edges can be:

                - 2-tuples (u,v) edge between u and v.
                - 3-tuples (u,v,k) where k is ignored.

        See Also
        --------
        remove_edge : remove a single edge
            
        Notes
        -----
        Will fail silently if an edge in ebunch is not in the graph.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> ebunch=[(1,2),(2,3)]
        >>> G.remove_edges_from(ebunch) 
        """
        for e in ebunch:
            u,v = e[:2]  # ignore edge data if present
            if u in self.adj and v in self.adj[u]:
                del self.adj[u][v]   
                if u != v:  # self loop needs only one entry removed
                    del self.adj[v][u]   


    def has_edge(self, u, v):
        """Return True if the edge (u,v) is in the graph.

        Parameters
        ----------
        u,v : nodes
            Nodes can be, for example, strings or numbers. 
            Nodes must be hashable (and not None) Python objects.
            
        Returns
        -------
        edge_ind : bool
            True if edge is in the graph, False otherwise.

        Examples
        --------
        Can be called either using two nodes u,v or edge tuple (u,v)

        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.has_edge(0,1)  # using two nodes
        True
        >>> e = (0,1)
        >>> G.has_edge(*e)  #  e is a 2-tuple (u,v)
        True
        >>> e = (0,1,{'weight':7})
        >>> G.has_edge(*e[:2])  # e is a 3-tuple (u,v,data_dictionary)
        True

        The following syntax are all equivalent: 
        
        >>> G.has_edge(0,1)
        True
        >>> 1 in G[0]  # though this gives KeyError if 0 not in G
        True
        
        """
        try:
            return v in self.adj[u]
        except KeyError:
            return False


    def neighbors(self, n):
        """Return a list of the nodes connected to the node n.

        Parameters
        ----------
        n : node
           A node in the graph

        Returns
        -------
        nlist : list
            A list of nodes that are adjacent to n.
        
        Raises
        ------
        NetworkXError
            If the node n is not in the graph.  

        Notes
        -----
        It is usually more convenient (and faster) to access the
        adjacency dictionary as G[n]:

        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edge('a','b',weight=7)
        >>> G['a']
        {'b': {'weight': 7}}

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.neighbors(0)
        [1]

        """
        try:
            return self.adj[n].keys()
        except KeyError:
            raise NetworkXError("The node %s is not in the graph."%(n,))

    def neighbors_iter(self, n):
        """Return an iterator over all neighbors of node n.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> print [n for n in G.neighbors_iter(0)]
        [1]

        Notes
        -----
        It is faster to use the idiom "in G[0]", e.g.
        >>> for n in G[0]:
        ...     print n
        1
        """
        try:
            return self.adj[n].iterkeys()
        except KeyError:
            raise NetworkXError("The node %s is not in the graph."%(n,))

    def edges(self, nbunch=None, data=False):
        """Return a list of edges.

        Edges are returned as tuples with optional data 
        in the order (node, neighbor, data).

        Parameters
        ----------
        nbunch : iterable container, optional (default= all nodes)
            A container of nodes.  The container will be iterated
            through once.
        data : bool, optional (default=False)
            Return two tuples (u,v) (False) or three-tuples (u,v,data) (True).

        Returns
        --------
        edge_list: list of edge tuples
            Edges that are adjacent to any node in nbunch, or a list
            of all edges if nbunch is not specified.

        See Also
        --------
        edges_iter : return an iterator over the edges

        Notes
        -----
        Nodes in nbunch that are not in the graph will be (quietly) ignored.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.edges()
        [(0, 1), (1, 2), (2, 3)]
        >>> G.edges(data=True) # default edge data is {} (empty dictionary)
        [(0, 1, {}), (1, 2, {}), (2, 3, {})]
        >>> G.edges([0,3])
        [(0, 1), (3, 2)]
        >>> G.edges(0)
        [(0, 1)]

        """
        return list(self.edges_iter(nbunch, data))

    def edges_iter(self, nbunch=None, data=False):
        """Return an iterator over the edges.
        
        Edges are returned as tuples with optional data 
        in the order (node, neighbor, data).

        Parameters
        ----------
        nbunch : iterable container, optional (default= all nodes)
            A container of nodes.  The container will be iterated
            through once.
        data : bool, optional (default=False)
            If True, return edge attribute dict in 3-tuple (u,v,data).

        Returns
        -------
        edge_iter : iterator
            An iterator of (u,v) or (u,v,d) tuples of edges.

        See Also
        --------
        edges : return a list of edges

        Notes
        -----
        Nodes in nbunch that are not in the graph will be (quietly) ignored.

        Examples
        --------
        >>> G = nx.Graph()   # or MultiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> [e for e in G.edges_iter()]
        [(0, 1), (1, 2), (2, 3)]
        >>> list(G.edges_iter(data=True)) # default data is {} (empty dict)
        [(0, 1, {}), (1, 2, {}), (2, 3, {})]
        >>> list(G.edges_iter([0,3]))
        [(0, 1), (3, 2)]
        >>> list(G.edges_iter(0))
        [(0, 1)]

        """
        seen={}     # helper dict to keep track of multiply stored edges
        if nbunch is None:
            nodes_nbrs = self.adj.iteritems()
        else:
            nodes_nbrs=((n,self.adj[n]) for n in self.nbunch_iter(nbunch))
        if data:
            for n,nbrs in nodes_nbrs:
                for nbr,data in nbrs.iteritems():
                    if nbr not in seen:
                        yield (n,nbr,data)
                seen[n]=1
        else:
            for n,nbrs in nodes_nbrs:
                for nbr in nbrs:
                    if nbr not in seen:
                        yield (n,nbr)
                seen[n] = 1
        del seen


    def get_edge_data(self, u, v, default=None):
        """Return the attribute dictionary associated with edge (u,v).

        Parameters
        ----------
        u,v : nodes
        default:  any Python object (default=None)           
            Value to return if the edge (u,v) is not found.  
            
        Returns
        -------
        edge_dict : dictionary
            The edge attribute dictionary.

        Notes
        -----
        It is faster to use G[u][v].

        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G[0][1]
        {}

        Warning: Assigning G[u][v] corrupts the graph data structure.
        But it is safe to assign attributes to that dictionary,

        >>> G[0][1]['weight'] = 7
        >>> G[0][1]['weight']
        7
        >>> G[1][0]['weight']
        7

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.get_edge_data(0,1) # default edge data is {}
        {}
        >>> e = (0,1)
        >>> G.get_edge_data(*e) # tuple form
        {}
        >>> G.get_edge_data('a','b',default=0) # edge not in graph, return 0
        0
        """
        try:
            return self.adj[u][v]
        except KeyError:
            return default

    def adjacency_list(self):
        """Return an adjacency list representation of the graph.

        The output adjacency list is in the order of G.nodes().
        For directed graphs, only outgoing adjacencies are included. 

        Returns
        -------
        adj_list : lists of lists
            The adjacency structure of the graph as a list of lists.

        See Also
        --------
        adjacency_iter

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.adjacency_list() # in order given by G.nodes()
        [[1], [0, 2], [1, 3], [2]]

        """
        return map(list,self.adj.itervalues())
        
    def adjacency_iter(self):
        """Return an iterator of (node, adjacency dict) tuples for all nodes.

        This is the fastest way to look at every edge. 
        For directed graphs, only outgoing adjacencies are included.

        Returns
        -------
        adj_iter : iterator
           An iterator of (node, adjacency dictionary) for all nodes in
           the graph.

        See Also
        --------
        adjacency_list
           
        Examples   
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> [(n,nbrdict) for n,nbrdict in G.adjacency_iter()]
        [(0, {1: {}}), (1, {0: {}, 2: {}}), (2, {1: {}, 3: {}}), (3, {2: {}})]

        """
        return self.adj.iteritems()

    def degree(self, nbunch=None, with_labels=False, weighted=False):
        """Return the degree of a node or nodes.

        The node degree is the number of edges adjacent to that node. 

        Parameters
        ----------
        nbunch : iterable container, optional (default=all nodes)
            A container of nodes.  The container will be iterated
            through once.    
        with_labels : bool, optional (default=False)
            If True return a dictionary of degrees keyed by node.
        weighted : bool, optional (default=False)
           If True return the sum of edge weights adjacent to the node.  

        Returns
        -------
        nd : list, or dictionary
            A list of node degrees or a dictionary with nodes as
            keys and degree as values if with_labels=True).

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.degree(0)
        1
        >>> G.degree([0,1])
        [1, 2]
        >>> G.degree([0,1],with_labels=True)
        {0: 1, 1: 2}

        """
        if with_labels:           # return a dict
            return dict(self.degree_iter(nbunch,weighted=weighted))
        elif nbunch in self:      # return a single node
            return self.degree_iter(nbunch,weighted=weighted).next()[1]
        else:                     # return a list
            return [d for (n,d) in self.degree_iter(nbunch,weighted=weighted)]

    def degree_iter(self, nbunch=None, weighted=False):
        """Return an iterator for (node, degree). 

        The node degree is the number of edges adjacent to the node. 

        Parameters
        ----------
        nbunch : iterable container, optional (default=all nodes)
            A container of nodes.  The container will be iterated
            through once.    
        weighted : bool, optional (default=False)
           If True return the sum of edge weights adjacent to the node.  

        Returns
        -------
        nd_iter : an iterator 
            The iterator returns two-tuples of (node, degree).
        
        See Also
        --------
        degree

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> list(G.degree_iter(0)) # node 0 with degree 1
        [(0, 1)]
        >>> list(G.degree_iter([0,1]))
        [(0, 1), (1, 2)]

        """
        if nbunch is None:
            nodes_nbrs = self.adj.iteritems()
        else:
            nodes_nbrs=((n,self.adj[n]) for n in self.nbunch_iter(nbunch))
  
        if weighted:                        
        # edge weighted graph - degree is sum of nbr edge weights
            for n,nbrs in nodes_nbrs:
                yield (n, sum((nbrs[nbr].get('weight',1) for nbr in nbrs)) +
                              (n in nbrs and nbrs[n].get('weight',1)))
        else:
            for n,nbrs in nodes_nbrs:
                yield (n,len(nbrs)+(n in nbrs)) # return tuple (n,degree)


    def clear(self):
        """Remove all nodes and edges from the graph.

        This also removes the name, and all graph, node, and edge attributes.
        
        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.clear()
        >>> G.nodes()
        []
        >>> G.edges()
        []

        """
        self.name = ''
        self.adj.clear() 
        self.node.clear()
        self.graph.clear()

    def copy(self):
        """Return a copy of the graph.

        Returns
        -------
        G : Graph
            A copy of the graph.   

        See Also
        --------
        to_directed: return a directed copy of the graph.

        Notes
        -----
        This makes a complete copy of the graph including all of the 
        node or edge attributes. 

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> H = G.copy()

        """
        return deepcopy(self)

    def is_multigraph(self):
        """Return True if graph is a multigraph, False otherwise."""
        return False


    def is_directed(self):
        """Return True if graph is directed, False otherwise."""
        return False

    def to_directed(self):
        """Return a directed representation of the graph.
 
        Returns
        -------
        G : DiGraph
            A directed graph with the same name, same nodes, and with
            each edge (u,v,data) replaced by two directed edges
            (u,v,data) and (v,u,data).

        Notes
        -----
        This is similar to DiGraph(self) which returns a shallow copy.  
        self.to_undirected() returns a deepcopy of edge, node and
        graph attributes.

        Examples
        --------
        >>> G = nx.Graph()   # or MultiGraph, etc
        >>> G.add_path([0,1])
        >>> H = G.to_directed()
        >>> H.edges()
        [(0, 1), (1, 0)]

        If already directed, return a (deep) copy

        >>> G = nx.DiGraph()   # or MultiDiGraph, etc
        >>> G.add_path([0,1])
        >>> H = G.to_directed()
        >>> H.edges()
        [(0, 1)]
        """
        from networkx import DiGraph 
        G=DiGraph()
        G.name=self.name
        G.add_nodes_from(self)
        G.add_edges_from( ((u,v,deepcopy(data)) 
                           for u,nbrs in self.adjacency_iter() 
                           for v,data in nbrs.iteritems()) )
        G.graph=deepcopy(self.graph)
        G.node=deepcopy(self.node)
        return G

    def to_undirected(self):
        """Return an undirected copy of the graph. 
 
        Returns
        -------
        G : Graph/MultiGraph
            A deepcopy of the graph.

        See Also
        --------
        copy, add_edge, add_edges_from

        Notes
        -----
        This is similar to Graph(self) which returns a shallow copy.  
        self.to_undirected() returns a deepcopy of edge, node and
        graph attributes.

        Examples
        --------
        >>> G = nx.Graph()   # or MultiGraph, etc
        >>> G.add_path([0,1])
        >>> H = G.to_directed()
        >>> H.edges()
        [(0, 1), (1, 0)]
        >>> G2 = H.to_undirected()
        >>> G2.edges()
        [(0, 1)]
        """
        return deepcopy(self)

    def subgraph(self, nbunch, copy=True):
        """Return the subgraph induced on nodes in nbunch.

        The induced subgraph of the graph has the nodes in nbunch 
        as its node set and the edges adjacent to those nodes as 
        its edge set.
        
        Parameters
        ----------
        nbunch : list, iterable
            A container of nodes.  The container will be iterated
            through once.    
        copy : bool, optional (default=True) 
            If True return a new graph holding the subgraph including
            copies of all edge and node properties.  If False the
            subgraph is created using the original graph by deleting
            all nodes not in nbunch (this changes the original graph).

        Returns
        -------
        G : Graph
            A subgraph of the graph.  If copy=True a new graph is
            returned with copies of graph, node, and edge data.  If
            copy=False the subgraph is created in place by modifying
            the original graph.

        Notes
        -----
        If copy=True, nodes and edges are copied using copy.deepcopy.

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> H = G.subgraph([0,1,2])
        >>> print H.edges()
        [(0, 1), (1, 2)]
        """
        bunch =set(self.nbunch_iter(nbunch))

        if not copy: 
            # remove all nodes (and attached edges) not in nbunch
            self.remove_nodes_from([n for n in self if n not in bunch])
            self.name = "Subgraph of (%s)"%(self.name)
            return self
        else:
            # create new graph and copy subgraph into it       
            H = self.__class__()
            H.name = "Subgraph of (%s)"%(self.name)
            # add nodes
            H.add_nodes_from(bunch)
            # add edges
            seen=set()
            for u,nbrs in self.adjacency_iter():
                if u in bunch:
                    for v,datadict in nbrs.iteritems():
                        if v in bunch and v not in seen:
                            dd=deepcopy(datadict)
                            H.adj[u][v]=dd
                            H.adj[v][u]=dd
                    seen.add(u)
            # copy node and graph attr dicts
            H.node=dict( (n,deepcopy(d)) 
                         for (n,d) in self.node.iteritems() if n in H)
            H.graph=deepcopy(self.graph)
            return H


    def nodes_with_selfloops(self):
        """Return a list of nodes with self loops.

        A node with a self loop has an edge with both ends adjacent
        to that node.

        Returns
        -------
        nodelist : list
            A list of nodes with self loops.

        See Also
        --------
        selfloop_edges, number_of_selfloops

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edge(1,1)
        >>> G.add_edge(1,2)
        >>> G.nodes_with_selfloops()
        [1]
        """
        return [ n for n,nbrs in self.adj.iteritems() if n in nbrs ]

    def selfloop_edges(self, data=False):
        """Return a list of selfloop edges.

        A selfloop edge has the same node at both ends.

        Parameters
        -----------
        data : bool, optional (default=False)
            Return selfloop edges as two tuples (u,v) (data=False)
            or three-tuples (u,v,data) (data=True)

        Returns
        -------
        edgelist : list of edge tuples
            A list of all selfloop edges.

        See Also
        --------
        selfloop_nodes, number_of_selfloops

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edge(1,1)
        >>> G.add_edge(1,2)
        >>> G.selfloop_edges()
        [(1, 1)]
        >>> G.selfloop_edges(data=True)
        [(1, 1, {})]
        """
        if data:
            return [ (n,n,nbrs[n]) 
                     for n,nbrs in self.adj.iteritems() if n in nbrs ]
        else:
            return [ (n,n) 
                     for n,nbrs in self.adj.iteritems() if n in nbrs ]


    def number_of_selfloops(self):
        """Return the number of selfloop edges.

        A selfloop edge has the same node at both ends.

        Returns
        -------
        nloops : int
            The number of selfloops.   

        See Also
        --------
        selfloop_nodes, selfloop_edges

        Examples
        --------
        >>> G=nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edge(1,1)
        >>> G.add_edge(1,2)
        >>> G.number_of_selfloops()
        1
        """
        return len(self.selfloop_edges())


    def size(self, weighted=False):
        """Return the number of edges.

        Parameters
        ----------
        weighted : boolean, optional (default=False)
           If True return the sum of the edge weights.

        Returns
        -------
        nedges : int
            The number of edges in the graph.

        See Also
        --------
        number_of_edges

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.size()
        3

        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_edge('a','b',weight=2)
        >>> G.add_edge('b','c',weight=4)
        >>> G.size()
        2
        >>> G.size(weighted=True)
        6
        """

        return sum(self.degree(weighted=weighted))/2

    def number_of_edges(self, u=None, v=None):
        """Return the number of edges between two nodes.

        Parameters
        ----------
        u,v : nodes, optional (default=all edges)
            If u and v are specified, return the number of edges between 
            u and v. Otherwise return the total number of all edges.
            
        Returns
        -------
        nedges : int
            The number of edges in the graph.  If nodes u and v are specified
            return the number of edges between those nodes.

        See Also
        --------
        size
            
        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.number_of_edges()
        3
        >>> G.number_of_edges(0,1) 
        1
        >>> e = (0,1)
        >>> G.number_of_edges(*e)
        1
        """
        if u is None: return self.size()
        if v in self.adj[u]:
            return 1
        else:
            return 0


    def add_star(self, nlist, **attr):
        """Add a star.

        The first node in nlist is the middle of the star.  It is connected 
        to all other nodes in nlist.

        Parameters
        ----------
        nlist : list 
            A list of nodes. 
        attr : keyword arguments, optional (default= no attributes)
            Attributes to add to every edge in star.

        See Also
        --------
        add_path, add_cycle

        Examples
        --------
        >>> G = nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_star([0,1,2,3])
        >>> G.add_star([10,11,12],weight=2)

        """
        v=nlist[0]
        edges=((v,n) for n in nlist[1:])
        self.add_edges_from(edges, **attr)

    def add_path(self, nlist, **attr):
        """Add a path.
        
        Parameters
        ----------
        nlist : list 
            A list of nodes.  A path will be constructed from
            the nodes (in order) and added to the graph.
        attr : keyword arguments, optional (default= no attributes)
            Attributes to add to every edge in path.

        See Also
        --------
        add_star, add_cycle

        Examples
        --------
        >>> G=nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_path([0,1,2,3])
        >>> G.add_path([10,11,12],weight=7)

        """
        edges=zip(nlist[:-1],nlist[1:])
        self.add_edges_from(edges, **attr)

    def add_cycle(self, nlist, **attr):
        """Add a cycle.
        
        Parameters
        ----------
        nlist : list 
            A list of nodes.  A cycle will be constructed from
            the nodes (in order) and added to the graph.
        attr : keyword arguments, optional (default= no attributes)
            Attributes to add to every edge in cycle.

        See Also
        --------
        add_path, add_star

        Examples
        --------
        >>> G=nx.Graph()   # or DiGraph, MultiGraph, MultiDiGraph, etc
        >>> G.add_cycle([0,1,2,3])
        >>> G.add_cycle([10,11,12],weight=7)

        """
        edges=zip(nlist,nlist[1:]+[nlist[0]])
        self.add_edges_from(edges, **attr)


    def nbunch_iter(self, nbunch=None):
        """Return an iterator of nodes contained in nbunch that are 
        also in the graph.

        The nodes in nbunch are checked for membership in the graph
        and if not are silently ignored.

        Parameters
        ----------
        nbunch : iterable container, optional (default=all nodes)
            A container of nodes.  The container will be iterated
            through once.    

        Returns
        -------
        niter : iterator
            An iterator over nodes in nbunch that are also in the graph.
            If nbunch is None, iterate over all nodes in the graph.

        Raises
        ------
        NetworkXError
            If nbunch is not a node or or sequence of nodes.
            If a node in nbunch is not hashable.

        See Also
        --------
        Graph.__iter__

        Notes 
        -----
        When nbunch is an iterator, the returned iterator yields values 
        directly from nbunch, becoming exhausted when nbunch is exhausted.
        
        To test whether nbunch is a single node, one can use 
        "if nbunch in self:", even after processing with this routine.
        
        If nbunch is not a node or a (possibly empty) sequence/iterator
        or None, a NetworkXError is raised.  Also, if any object in
        nbunch is not hashable, a NetworkXError is raised.
        """
        if nbunch is None:   # include all nodes via iterator
            bunch=self.adj.iterkeys()
        elif nbunch in self: # if nbunch is a single node 
            bunch=[nbunch].__iter__()
        else:                # if nbunch is a sequence of nodes
            def bunch_iter(nlist,adj):
                try:    
                    for n in nlist:
                        if n in adj:
                            yield n
                except TypeError,e:
                    message=e.args[0]
                    print message
                    # capture error for non-sequence/iterator nbunch.
                    if 'iterable' in message:  
                        raise NetworkXError(
                            "nbunch is not a node or a sequence of nodes.")
                    # capture error for unhashable node.
                    elif 'hashable' in message: 
                        raise NetworkXError(
                            "Node %s in the sequence nbunch is not a valid node."%n)
                    else: raise
            bunch=bunch_iter(nbunch,self.adj)
        return bunch
