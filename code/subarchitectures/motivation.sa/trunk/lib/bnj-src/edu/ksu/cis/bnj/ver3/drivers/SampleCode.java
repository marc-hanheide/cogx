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
 */package edu.ksu.cis.bnj.ver3.drivers;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.Field;
/**
 * file: SampleCode.java
 * 
 * @author Jeffrey M. Barber
 */
public class SampleCode
{
	void HowToCreateABayesianNetwork()
	{
		// create the network
		BeliefNetwork bn = new BeliefNetwork("A Sample Network");
		// create a boolean domain
		Discrete Bool = new Discrete(new String[] { "True", "False" });
		// create two nodes
		BeliefNode A = new BeliefNode("A", Bool);
		BeliefNode B = new BeliefNode("B", Bool);
		// insert the nodes into the
		bn.addBeliefNode(A);
		bn.addBeliefNode(B);
		// connect from A to B
		bn.connect(A, B);
	}
	void HowToNavigateABayesianNetwork(BeliefNetwork bn)
	{
		BeliefNode[] Nodes = bn.getNodes();
		for (int i = 0; i < Nodes.length; i++)
		{
			BeliefNode[] Children = bn.getChildren(Nodes[i]);
			BeliefNode[] Parents = bn.getParents(Nodes[i]);
			// do stuff
		}
	}
	boolean[]	Mark	= null;
	void DFS(BeliefNetwork bn, BeliefNode X)
	{
		// already seen this node
		if (Mark[X.loc()]) return;
		// mark node as seen
		Mark[X.loc()] = true;
		// get the Children
		BeliefNode[] Children = bn.getChildren(X);
		// for each child
		for (int i = 0; i < Children.length; i++)
		{
			DFS(bn, Children[i]);
		}
	}
	void DepthFirstSearch(BeliefNetwork bn)
	{
		// get each node
		BeliefNode[] Nodes = bn.getNodes();
		// create the markings
		Mark = new boolean[Nodes.length];
		// initialize the mark
		for (int i = 0; i < Mark.length; i++)
		{
			Mark[i] = false;
		}
		// for each node
		for (int i = 0; i < Nodes.length; i++)
		{
			DFS(bn, Nodes[i]);
		}
	}
	
	
	void OperateOnValues(Value a,Value b)
	{
		Value c;
		// c = a + b
		c = Field.add(a,b);
		// c = a - b
		c = Field.subtract(a,b);
		// c = a * b
		c = Field.mult(a,b);
		// c = a / b
		c = Field.divide(a,b);
	}
	
	void HowToIterateCPF(CPF X)
	{
		// Fastest way!
		for(int i = 0; i < X.size(); i++)
		{
			// get(i)
			// put(i, V)
		}
		
		// Need the logical address?
		// Slow Way
		for(int i = 0; i < X.size(); i++)
		{
			int[] q = X.realaddr2addr(i); // SLOW
			// do stuff with q
			// get(i)
			// put(i, V)
		}
		
		// Fast Way
		int[] q = X.realaddr2addr(0); // SLOW
		for(int i = 0; i < X.size(); i++)
		{
			// do stuff with q
			// get(i)
			// put(i, V)
			X.addOne(q);
		}
	}
	
	void HowToOperateDifferentDomains(CPF Large, CPF Small)
	{
		// The slow way
		int[] q = Large.realaddr2addr(0);
		for(int i = 0; i < Large.size(); i++)
		{
			int[] nQ = CPF.getSubQuery(q, Large.getDomainProduct(), Small.getDomainProduct());
			// nQ is now an address in the Small CPF, which we could use in various ways
			Large.addOne(q);
		}
		
		
		// the faster way
		//int[] q = Large.realaddr2addr(0);
		int[] proj = CPF.getProjectionMapping(Large.getDomainProduct(), Small.getDomainProduct());
		int[] nQ = Small.realaddr2addr(0);
		for(int i = 0; i < Large.size(); i++)
		{
			CPF.applyProjectionMapping(q,proj,nQ);
			// nQ is now an address in the Small CPF, which we could use in various ways
			Large.addOne(q);
		}
	}
}