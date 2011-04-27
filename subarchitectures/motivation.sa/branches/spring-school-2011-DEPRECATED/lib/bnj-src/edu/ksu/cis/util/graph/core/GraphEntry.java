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
 * \file GraphEntry.java
 * \author Jeffrey M. Barber
 */
public class GraphEntry
{
	private Vertex		_VertexEntry;
	private int			_Children[];
	private int			_Parents[];
	private int			_ChildrenSize;
	private int			_ParentsSize;
	private Vertex[]	_ParentsVertexCache;
	private Vertex[]	_ChildrenVertexCache;
	private boolean		_ParentsCacheIsDirty;
	private boolean		_ChildrensCacheIsDirty;
	/*! get the vertex
	 * \return the vertex of this entry
	 */
	Vertex v()
	{
		return _VertexEntry;
	}
	/*! Construct a Graph Entry associated to a vertex
	 * \param[in] V - The associated vertex for this entry in the graph
	 */
	GraphEntry(Vertex V)
	{
		_VertexEntry = V;
		_ChildrenSize = 0;
		_ParentsSize = 0;
		_Children = new int[10];
		_Parents = new int[10];
		_ParentsCacheIsDirty = true;
		_ChildrensCacheIsDirty = true;
	}
	/*! [cache managment]
	 * \param[in] b - the additional ammount of memory wanted to extend the buffer
	 */
	private void buffer(int b)
	{
		int[] nc = new int[_Children.length + b];
		int[] np = new int[_Parents.length + b];
		for (int i = 0; i < _ChildrenSize; i++)
		{
			nc[i] = _Children[i];
		}
		for (int i = 0; i < _ParentsSize; i++)
		{
			np[i] = _Parents[i];
		}
		_Children = nc;
		_Parents = np;
	}
	/*! Reduce wasted memory for the adj info, increases locality in ideal situations
	 */
	void Compact()
	{
		int[] nc = new int[_ChildrenSize];
		int[] np = new int[_ParentsSize];
		for (int i = 0; i < _ChildrenSize; i++)
		{
			nc[i] = _Children[i];
		}
		for (int i = 0; i < _ParentsSize; i++)
		{
			np[i] = _Parents[i];
		}
		_Children = nc;
		_Parents = np;
		_ParentsCacheIsDirty = true;
		_ChildrensCacheIsDirty = true;
	}
	/*! Swaps two elements in an integer array
	 * \param[in] a integer array
	 * \param[in] i first element
	 * \param[in] j second element
	 */
	private void swap(int[] a, int i, int j)
	{
		int t = a[i];
		a[i] = a[j];
		a[j] = t;
	}
	/*! Maps the parent of the node [col of adj matrix]
	 * \param[in] par
	 */
	void mapParent(int par)
	{
		if (canNavigatedByParent(par)) return;
		if (_Parents.length - 2 < _ParentsSize) buffer(5);
		_Parents[_ParentsSize] = par;
		int i = _ParentsSize;
		while (i >= 1 && _Parents[i] < _Parents[i - 1])
		{
			swap(_Parents, i, i - 1);
			i--;
		}
		_ParentsCacheIsDirty = true;
		_ParentsSize++;
	}
	/*! maps a child of this node [row of adj matrix]
	 * \param[in] chi
	 */
	void mapChild(int chi)
	{
		if (canNavigateByChild(chi)) return;
		if (_Children.length - 2 < _ChildrenSize) buffer(5);
		_Children[_ChildrenSize] = chi;
		int i = _ChildrenSize;
		while (i >= 1 && _Children[i] < _Children[i - 1])
		{
			swap(_Children, i, i - 1);
			i--;
		}
		_ChildrensCacheIsDirty = true;
		_ChildrenSize++;
	}
	/*! the inverse of mapParent
	 * \param[in] par
	 */
	void unmapParent(int par)
	{
		for (int i = 0; i < _ParentsSize; i++)
		{
			if (par == _Parents[i])
			{
				_Parents[i] = -1;
			}
		}
		int b = 0;
		for (int i = 0; i < _ParentsSize; i++)
		{
			_Parents[i - b] = _Parents[i];
			if (_Parents[i] == -1) b++;
		}
		_ParentsSize -= b;
		_ParentsCacheIsDirty = true;
	}
	/*! empty the entry
	 */
	void empty()
	{
		_ChildrenSize = 0;
		_ChildrensCacheIsDirty = true;
		_ParentsSize = 0;
		_ParentsCacheIsDirty = true;
	}
	/*! the inverse of mapChild
	 * \param[in] chi
	 */
	void unmapChild(int chi)
	{
		for (int i = 0; i < _ChildrenSize; i++)
		{
			if (chi == _Children[i]) _Children[i] = -1;
		}
		int b = 0;
		for (int i = 0; i < _ChildrenSize; i++)
		{
			_Children[i - b] = _Children[i];
			if (_Children[i] == -1) b++;
		}
		_ChildrenSize -= b;
		_ChildrensCacheIsDirty = true;
	}
	/*! Reorders the vertice cache
	 * \param[in] vold - the original vertex
	 * \param[in] vnew - the new vertex
	 */
	void remap(int vold, int vnew)
	{
		if (canNavigatedByParent(vold))
		{
			unmapParent(vold);
			mapParent(vnew);
		}
		if (canNavigateByChild(vold))
		{
			unmapChild(vold);
			mapChild(vnew);
		}
	}
	/*! Return the parents of this node as vertex array given the whole cache 
	 * \param[in] total - the store for rebuilding the cache
	 * return an array of vertices (the parents)
	 */
	Vertex[] getParents(GraphEntry[] total)
	{
		if (_ParentsCacheIsDirty)
		{
			_ParentsVertexCache = new Vertex[_ParentsSize];
			for (int i = 0; i < _ParentsSize; i++)
				_ParentsVertexCache[i] = total[_Parents[i]].v();
			_ParentsCacheIsDirty = false;
		}
		return _ParentsVertexCache;
	}
	/*! Return the children of this node as vertex array given the whole cache 
	 * \param[in] total - the store for rebuilding the cache
	 * \return an array of vertices (the children)
	 */
	Vertex[] getChildren(GraphEntry[] total)
	{
		if (_ChildrensCacheIsDirty)
		{
			_ChildrenVertexCache = new Vertex[_ChildrenSize];
			for (int i = 0; i < _ChildrenSize; i++)
				_ChildrenVertexCache[i] = total[_Children[i]].v();
			_ChildrensCacheIsDirty = false;
		}
		return _ChildrenVertexCache;
	}
	/*! Get the number of Parents
	 * \return the number of parents [non zero entries in the col of the adj matrix]
	 */
	int parentsSize()
	{
		return _ParentsSize;
	}
	/*! Get the number of Children
	 * \return the number of children [non zero entries in the row of the adj matrix]
	 */
	int childrenSize()
	{
		return _ChildrenSize;
	}
	/*! Can this vertex navigigate to a node if the node is a child
	 * \param[in] c
	 * \return answer to question
	 */
	boolean canNavigateByChild(int c)
	{
		int high = _ChildrenSize;
		int low = -1;
		while ((high - low) > 1)
		{
			int p = (high + low) / 2;
			if (c < _Children[p])
			{
				high = p;
			}
			else if (_Children[p] < c)
			{
				low = p;
			}
			else
				return true;
		}
		if (low >= 0 && _Children[low] == c) return true;
		return false;
	}
	/*! Can this vertex navigigate to a node if the node is a parent
	 * \param[in] c
	 * \return answer to question
	 */
	boolean canNavigatedByParent(int c)
	{
		int high = _ParentsSize;
		int low = -1;
		while ((high - low) > 1)
		{
			int p = (high + low) / 2;
			if (c < _Parents[p])
			{
				high = p;
			}
			else if (_Parents[p] < c)
			{
				low = p;
			}
			else
				return true;
		}
		if (low >= 0 && _Parents[low] == c) return true;
		return false;
	}
	/*! application of an ordering to the children
	 * \param[in] order element in permutation group
	 */
	void applyOrdering(int[] order)
	{
		int i;
		for (i = 0; i < _ChildrenSize; i++)
		{
			_Children[i] = order[_Children[i]];
		}
		for (i = 0; i < _ParentsSize; i++)
		{
			_Parents[i] = order[_Parents[i]];
		}
		_ParentsCacheIsDirty = true;
		_ChildrensCacheIsDirty = true;
	}
}