/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.util.graph.core;
/*!
 * \file Graph.java
 * \author Jeffrey M. Barber
 */
public class Graph
{
	private GraphEntry[]	_VertexStore;
	private Object[]		_ObjectCopyMap;
	private int				_VertexCount;
	private int				_EdgeCount;
	/*! Construct an empty graph
	 */
	public Graph()
	{
		_VertexCount = 0;
		_VertexStore = new GraphEntry[1000];
		_EdgeCount = 0;
	}
	/*! Buffer elements for insertion into Store
	 * \param[in] b the number of new elements
	 */
	private void buffer(int b)
	{
		GraphEntry[] ne = new GraphEntry[b + _VertexStore.length];
		for (int i = 0; i < _VertexCount; i++)
			ne[i] = _VertexStore[i];
		_VertexStore = ne;
	}
	/*! Compact the storage for minimal usage
	 */
	public void Compact()
	{
		GraphEntry[] ne = new GraphEntry[_VertexStore.length];
		for (int i = 0; i < _VertexCount; i++)
		{
			_VertexStore[i].Compact();
			ne[i] = _VertexStore[i];
		}
		_VertexStore = ne;
	}
	/*! Translates all the vertices
	 * !param[in] x the x offset
	 * !param[in] y the y offset
	 */
	public void translate(int x, int y)
	{
		for (int i = 0; i < _VertexCount; i++)
		{
			_VertexStore[i].v().translate(x, y);
		}
	}
	/*! Insert the vertex into the list
	 * \param[in] V
	 */
	public void addVertex(Vertex V)
	{
		if (V == null) return;
		if (V.loc() >= 0) return;
		GraphEntry GE = new GraphEntry(V);
		if (_VertexStore.length < _VertexCount + 1) buffer(10);
		_VertexStore[_VertexCount] = GE;
		V.setLOC(_VertexCount);
		_VertexCount++;
	}
	/*! Removes the vertex from the store [VERY EXPENSIVE]
	 * \param[in] V the vertex to remove
	 */
	public void removeVertex(Vertex V)
	{
		if (V == null) return;
		if (_VertexStore[V.loc()].v() != V) return;
		int kVloc = V.loc();
		for (int i = 0; i < _VertexCount; i++)
		{
			_VertexStore[i].unmapParent(kVloc);
			_VertexStore[i].unmapChild(kVloc);
		}
		for (int i = V.loc(); i < _VertexCount - 1; i++)
		{
			_VertexStore[i] = _VertexStore[i + 1];
		}
		_VertexStore[_VertexCount - 1] = null;
		_VertexCount--;
		for (int i = 0; i < _VertexCount; i++)
		{
			for (int j = V.loc(); j < _VertexCount; j++)
			{
				_VertexStore[i].remap(j + 1, j);
			}
			_VertexStore[i].v().setLOC(i);
		}
		V.setLOC(-1);
	}
	/*! Inserts a general edge connecting two vertices
	 * \param[in] E
	 */
	public void addEdge(Edge E)
	{
		_VertexStore[E.s().loc()].mapChild(E.d().loc());
		_VertexStore[E.d().loc()].mapParent(E.s().loc());
		_EdgeCount++;
		if (!E.isDirected())
		{
			_VertexStore[E.s().loc()].mapParent(E.d().loc());
			_VertexStore[E.d().loc()].mapChild(E.s().loc());
			_EdgeCount++;
		}
	}
	/*! Create (src,dst) in the edge set
	 * \param[in] Src the Parent
	 * \param[in] Dst the Child
	 */
	public void addDirectedEdge(Vertex Src, Vertex Dst)
	{
		addEdge(new Edge(Src, Dst));
	}
	/*! Add (a,b) and (b,a) to the edge set
	 * \param[in] A a vertex to/from B
	 * \param[in] B a vertex to/from B
	 */
	public void addUndirectedEdge(Vertex A, Vertex B)
	{
		Edge E = new Edge(A, B);
		E.setDirected(false);
		addEdge(E);
	}
	/*! Removes a general edge from the edge set
	 * \param[in] E
	 */
	public void removeEdge(Edge E)
	{
		removeEdge(E.s(), E.d());
		if (!E.isDirected()) removeEdge(E.d(), E.s());
	}
	/*! removes (src,dst) from the edge set
	 * \param[in] Src
	 * \param[in] Dst
	 */
	public void removeEdge(Vertex Src, Vertex Dst)
	{
		_VertexStore[Src.loc()].unmapChild(Dst.loc());
		_VertexStore[Dst.loc()].unmapChild(Src.loc());
		_VertexStore[Src.loc()].unmapParent(Dst.loc());
		_VertexStore[Dst.loc()].unmapParent(Src.loc());
		RecalculateEdges();
	}
	/*! Applies a Graph Visitor to the graph
	 * \param[in] GV the visitor
	 * \param[in] donodes - should the nodes be visited?
	 * \param[in] doedges - should the edges be visited?
	 */
	public void apply(Visitor GV, boolean donodes, boolean doedges)
	{
		if (donodes)
		{
			for (int i = 0; i < _VertexCount; i++)
			{
				GV.onVertex(_VertexStore[i].v());
			}
		}
		if (doedges)
		{
			Vertex[] child;
			for (int i = 0; i < _VertexCount; i++)
			{
				child = _VertexStore[i].getChildren(_VertexStore);
				//.getChildren(_entries);
				int nc = _VertexStore[i].childrenSize();
				for (int j = 0; j < nc; j++)
				{
					Edge E = new Edge(_VertexStore[i].v(), child[j]);
					if (getConnectedness(E.s(), E.d()) == 2)
					{
						E.setDirected(false);
					}
					GV.onEdge(E);
				}
			}
		}
	}
	/*! Get the parents of a vertex
	 * \param[in] V the vertex of the query
	 * \return the parents of a vertex
	 */
	public Vertex[] getParents(Vertex V)
	{
		int i = V.loc();
		return _VertexStore[i].getParents(_VertexStore);
	}
	/*! Gets the children of a vertex
	 * \param[in] V the vertex of the query
	 * \return the children of a vertex
	 */
	public Vertex[] getChildren(Vertex V)
	{
		int i = V.loc();
		return _VertexStore[i].getChildren(_VertexStore);
	}
	/*! Return the connectedness of vertices
	 * \param[in] V1 the first vertex
	 * \param[in] V2 the second vertex
	 * \return 2 if bidirectionaly, 1 is just (V1,V2) exists, -1 if (V1,V2) exists, 0 if not connected
	 */
	public int getConnectedness(Vertex V1, Vertex V2)
	{
		if (_VertexStore[V1.loc()].canNavigateByChild(V2.loc()))
		{
			if (_VertexStore[V2.loc()].canNavigateByChild(V1.loc())) return 2;
			return 1;
		}
		if (_VertexStore[V2.loc()].canNavigateByChild(V1.loc())) return -1;
		return 0;
	}
	public Object[] getObjectsAfterCopy()
	{
		return _ObjectCopyMap;
	}
	/*! returns a copy of the graph
	 * \return the graph copy()
	 */
	public Graph copy()
	{
		_ObjectCopyMap = new Object[_VertexCount];
		Graph DeepCopy = new Graph();
		Vertex[] _map = new Vertex[_VertexCount];
		for (int i = 0; i < _VertexCount; i++)
		{
			_map[i] = _VertexStore[i].v().copy();
			DeepCopy.addVertex(_map[i]);
			_ObjectCopyMap[i] = _VertexStore[i].v().getObject();
			if (_map[i].loc() != i)
			{
				// ASSUMPTION ERROR
				return null;
			}
		}
		Vertex[] child;
		for (int i = 0; i < _VertexCount; i++)
		{
			child = _VertexStore[i].getChildren(_VertexStore);
			if (child != null)
			{
				int nc = _VertexStore[i].childrenSize();
				for (int j = 0; j < nc; j++)
				{
					//Edge E = new Edge(_map[i], _map[child[j].loc()]);
					//DeepCopy.addEdge(E);
					DeepCopy.addDirectedEdge(_map[i], _map[child[j].loc()]);
				}
			}
		}
		return DeepCopy;
	}
	/*! For interation support
	 * \return the iterator
	 */
	public int begin()
	{
		return 0;
	}
	/*! Get the vertex at an iterator
	 * \param[in] enum
	 * \return the vertex
	 */
	public Vertex get(int enum)
	{
		if (0 <= enum && enum < _VertexCount)
		{
			Vertex T = _VertexStore[enum].v();
			return T;
		}
		return null;
	}
	/*! Detemines if an iterator should stop iterating
	 * \param[in] enum
	 * \return true/false if the iterator is bad
	 */
	public boolean end(int enum)
	{
		return enum < _VertexCount && enum >= 0;
	}
	/*! get the number of vertices 
	 * \return the number of vertices
	 */
	public int getNumberOfVertices()
	{
		return _VertexCount;
	}
	/*! get the degree of a vertex, both in and out
	 * \param[in] V the vertex of the query
	 * \return the number of edges to a vertex
	 */
	public int getDegree(Vertex V)
	{
		return _VertexStore[V.loc()].parentsSize() + _VertexStore[V.loc()].childrenSize();
	}
	/*! get the in degree of a vertex
	 * \param[in] V the vertex of the query
	 * \return the number of edges into a vertex
	 */
	public int getInDegree(Vertex V)
	{
		return _VertexStore[V.loc()].parentsSize();
	}
	/*! get the out degree of a vertex
	 * \param[in] V the vertex of the query
	 * \return the number of edges out a vertex
	 */
	public int getOutDegree(Vertex V)
	{
		return _VertexStore[V.loc()].childrenSize();
	}
	/*! Apply an ordering to the graph, maintaining all things
	 * \param order an array [0,N-1] inclusive where N = getNumberOfVertices()
	 */
	public void applyOrdering(int[] order)
	{
		GraphEntry[] cpy = new GraphEntry[_VertexStore.length];
		if (order.length != _VertexCount) return;
		for (int i = 0; i < _VertexCount; i++)
		{
			cpy[order[i]] = _VertexStore[i];
			cpy[order[i]].v().setLOC(order[i]);
			cpy[order[i]].applyOrdering(order);
		}
		_VertexStore = cpy;
	}
	/*! Get the vertices
	 * \return an array with all the vertices
	 */
	public Vertex[] getVertices()
	{
		Vertex[] V = new Vertex[_VertexCount];
		for (int i = 0; i < _VertexCount; i++)
		{
			V[i] = _VertexStore[i].v();
		}
		return V;
	}
	/*! Recalculate edges
	 */
	private void RecalculateEdges()
	{
		_EdgeCount = 0;
		for (int i = 0; i < _VertexCount; i++)
		{
			_EdgeCount += _VertexStore[i].childrenSize();
		}
	}
	/*! Get the number of Edges
	 * \return the number of edges in the graph
	 */
	public int getNumberOfEdges()
	{
		return _EdgeCount;
	}
}