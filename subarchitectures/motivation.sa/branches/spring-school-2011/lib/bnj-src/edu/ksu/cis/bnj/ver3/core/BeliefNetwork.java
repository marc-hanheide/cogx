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
 */package edu.ksu.cis.bnj.ver3.core;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Reader;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Writer;
import edu.ksu.cis.util.graph.core.*;
/*!
 * \file BeliefNetwork.java
 * \author Jeffrey M. Barber
 */
public class BeliefNetwork
{
	private Graph	_Graph;
	private String	_Name;
	/*!
	 * Constructs an empty beliefnetwork
	 */
	public BeliefNetwork()
	{
		_Graph = new Graph();
		_Name = "bn";
	}
	/*! Constructs an empty beliefnetwork with
	 * \param[in] name The name of the new network
	 */
	public BeliefNetwork(String name)
	{
		_Graph = new Graph();
		_Name = name;
	}
	/*!
	 * get the network's backing graph
	 * \note DO NOT USE unless you
	 * \par
	 * 	   plan to copy() it and change the structure from there
	 *  or plan not to modify it
	 * \return The graph associated to the belief network
	 */
	public Graph getGraph()
	{
		return _Graph;
	}
	/*!
	 * Sets the network's name
	 * \param[in] name The new name of the network
	 */
	public void setName(String name)
	{
		_Name = name;
	}
	/*!
	 * Gets the network's name
	 * \return the name of the network
	 */
	public String getName()
	{
		return _Name;
	}
	/*! Add a belief node to this network
	 * @param[in] bnode	The node to be inserted to the graph
	 */
	public void addBeliefNode(BeliefNode bnode)
	{
		Vertex V = new Vertex(bnode.getName());
		_Graph.addVertex(V);
		V.setObject(bnode);
		bnode.setOwner(V);
	}
	/*! Connect two nodes together
	 * @param[in] parent	 parent which the bnode will be a child of
	 * @param[in] bnode  	node which will be made a child of parent
	 */
	public void connect(BeliefNode parent, BeliefNode bnode)
	{
		_Graph.addDirectedEdge(parent.getOwner(), bnode.getOwner());
		Vertex[] parents = _Graph.getParents(bnode.getOwner());
		BeliefNode[] after = new BeliefNode[parents.length + 1];
		for (int i = 0; i < parents.length; i++)
		{
			after[i + 1] = ((BeliefNode) parents[i].getObject());
		}
		after[0] = bnode;
		CPF beforeCPF = bnode.getCPF();
		bnode.setCPF(beforeCPF.expand(after).hardcopy());
		int pSize2 = bnode.getCPF().size();
	}
	/*! Disconect two nodes, inverse of connect
	 * @param[in] parent	parent which the node will be cut off from
	 * @param[in] bnode		node to be cut from parent
	 */
	public void disconnect(BeliefNode parent, BeliefNode bnode)
	{
		_Graph.removeEdge(parent.getOwner(), bnode.getOwner());
		Vertex[] parents = _Graph.getParents(bnode.getOwner());
		BeliefNode[] after = new BeliefNode[parents.length + 1];
		for (int i = 0; i < parents.length; i++)
		{
			after[i + 1] = ((BeliefNode) parents[i].getObject());
		}
		after[0] = bnode;
		CPF beforeCPF = bnode.getCPF();
		bnode.setCPF(beforeCPF.extract(after));
		bnode.getCPF().normalizeByDomain();
	}
	/*! Delete a belief from network (does disconnects)
	 * 	@param[in] bnode 	the node to be removed
	 */
	public void deleteBeliefNode(BeliefNode bnode)
	{
		BeliefNode[] P = getParents(bnode);
		BeliefNode[] C = getChildren(bnode);
		for (int i = 0; i < P.length; i++)
			disconnect(P[0], bnode);
		for (int i = 0; i < C.length; i++)
			disconnect(bnode, C[i]);
		_Graph.removeVertex(bnode.getOwner());
	}
	/*! Change a belief's nodes domain (update cpf)
	 * @param[in] bnode		the node to update
	 * @param[in] domNew 	the new domain
	 */
	public void changeBeliefNodeDomain(BeliefNode bnode, Domain domNew)
	{
		int[] domainMap = new int[bnode.getDomain().getOrder() + domNew.getOrder()];
		for (int i = 0; i < bnode.getDomain().getOrder(); i++)
		{
			domainMap[i] = -1;
			for (int k = 0; k < domNew.getOrder(); k++)
			{
				if (domNew.getName(k).equals(bnode.getDomain().getName(i)))
				{
					domainMap[i] = k;
				}
			}
		}
		for (int i = bnode.getDomain().getOrder(); i < bnode.getDomain().getOrder() + domNew.getOrder(); i++)
		{
			domainMap[i] = i;
		}
		BeliefNode[] children = getChildren(bnode);
		for (int i = 0; i < children.length; i++)
		{
			CPF mod = CPF.changeDomain(children[i].getCPF(), bnode, domNew, domainMap);
			children[i].setCPF(mod);
		}
		bnode.setCPF(CPF.changeDomain(bnode.getCPF(), bnode, domNew, domainMap));
		bnode.setDomain(domNew);
	}
	/*! Get the nodes
	 * @return An array of all the nodes in the network [such that BeliefNode::loc() = position in array]
	 */
	public BeliefNode[] getNodes()
	{
		Vertex[] V = _Graph.getVertices();
		BeliefNode[] nodes = new BeliefNode[V.length];
		for (int i = 0; i < V.length; i++)
		{
			nodes[i] = (BeliefNode) V[i].getObject();
		}
		return nodes;
	}
	/*! Get the parents of the node
	 * 
	 * @param[in] bnode The node for who we request the parents of
	 * @return	An array of all the parents of bnode
	 */
	public BeliefNode[] getParents(BeliefNode bnode)
	{
		Vertex[] vParents = _Graph.getParents(bnode.getOwner());
		if (vParents == null) return null;
		BeliefNode[] parents = new BeliefNode[vParents.length];
		for (int i = 0; i < vParents.length; i++)
		{
			parents[i] = (BeliefNode) vParents[i].getObject();
		}
		return parents;
	}
	/*! Get the children of a node
	 * 
	 * @param[in] bnode The node for who we request the children of
	 * @return	An array of all the children of bnode
	 */
	public BeliefNode[] getChildren(BeliefNode bnode)
	{
		Vertex[] vChildren = _Graph.getChildren(bnode.getOwner());
		if (vChildren == null) return null;
		BeliefNode[] children = new BeliefNode[vChildren.length];
		for (int i = 0; i < vChildren.length; i++)
		{
			children[i] = (BeliefNode) vChildren[i].getObject();
		}
		return children;
	}
	/*! Apply an ordering to the noders
	 * param[in] ordering 	The order the nodes, ordering is a permutation of [0,N-1] where N = ordering.length
	 */
	public void applyOrder(int[] ordering)
	{
		_Graph.applyOrdering(ordering);
	}
	/*! Clone this BeliefNetwork, (a structure and data clone)
	 * @return a copy of the network
	 */
	public BeliefNetwork copy()
	{
		OmniFormatV1_Reader cheat = new OmniFormatV1_Reader();
		OmniFormatV1_Writer.Write(this, cheat);
		return cheat.GetBeliefNetwork(0);
	}
	/*!
	 * \return the answer to the question
	 */
	public boolean isInfluenceDiagram()
	{
		BeliefNode[] nodes = getNodes();
		for(int i = 0; i < nodes.length; i++)
		{
			if(nodes[i].getType() != BeliefNode.NODE_CHANCE)
				return true;
		}
		return false;
	}
	
	public BeliefNode findNode(String x)
	{
	    BeliefNode[] nodes = getNodes();
	    for(int k = 0; k < nodes.length; k++)
	        if(nodes[k].getName().equals(x))
	            return nodes[k];
	    return null;
	}
}