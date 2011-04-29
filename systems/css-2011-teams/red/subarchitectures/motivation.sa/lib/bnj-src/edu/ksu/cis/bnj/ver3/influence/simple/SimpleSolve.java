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
 */package edu.ksu.cis.bnj.ver3.influence.simple;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.Field;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueUnity;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.bnj.ver3.inference.exact.LSonline;
import edu.ksu.cis.bnj.ver3.influence.Solver;
import edu.ksu.cis.bnj.ver3.influence.nfseq.Preconditions;

/*!
 * \file SimpleSolve.java
 * \author Jeffrey M. Barber
 */
public class SimpleSolve implements Solver
{
	int[]					_QueryMap;
	BeliefNode[]			_UtilityNodes;
	BeliefNode[]			_DescisionNodes;
	// cache
	Inference				inferenceMethod;
	private BeliefNetwork	_TransformedNetwork;
	private BeliefNetwork	_OriginalNetwork;
	/*! What is the name of this influence diagram solver
	 * \return the name
	 */
	public String getName()
	{
		return "Simple Influence Diagram Solver by Enumeration";
	}

	public SimpleSolve()
	{
	}

	private void cpfFill(CPF X, int[] q, int k, CPF[] marginals, Value carry)
	{
		if(k >= X.getDomainProduct().length)
		{
			X.put(q,carry);
			return;
		}
		for(int s = 0; s < marginals[k].size(); s++)
		{
			Value m = marginals[k].get(s);
			q[k] = s;
			cpfFill(X, q, k+1, marginals, Field.mult(carry, m));
		}
	}
	
	private CPF getDirectProb(CPF[] Marginals, BeliefNode[] Parents)
	{
		ValueUnity unity = new ValueUnity();
		CPF X = new CPF(Parents);
		int[] q = X.realaddr2addr(0);
		cpfFill(X, q, 0, Marginals, unity);
		return X;
	}
	
	private Value evalUtilityOn(BeliefNode bnode)
	{
		BeliefNode[] parents = _OriginalNetwork.getParents(bnode);
		CPF[] marginals = new CPF[parents.length];
		for(int i = 0; i < parents.length; i++)
		{
			marginals[i] = queryMarginal(parents[i]);
		}
		CPF R = getDirectProb(marginals, parents);
		R = CPF.multiply(R,bnode.getCPF());
		Value S = new ValueZero();
		for(int i = 0; i < R.size(); i++)
		{
			S = Field.add(S, R.get(i));
		}
		return S;
	}
	
	private Value evalUtility()
	{
		Value T = new ValueZero();
		for(int k = 0; k < _UtilityNodes.length; k++)
		{
			Value U = evalUtilityOn(_UtilityNodes[k]);
			T = Field.add(T,U);
		}
		return T;
	}
	/*! Solve this network
	 * 
	 * \param[in] bn the Bayesian Belief Network
	 */
	public void solve(BeliefNetwork bn)
	{
		// update reference
		_OriginalNetwork = bn;
		// set our inference method
		inferenceMethod = new LSonline();
		///// Build the transformed network
		_TransformedNetwork = new BeliefNetwork();
		// count the chance + desc nodes, build mapping
		BeliefNode[] OriginalNodes = bn.getNodes();

		// mapping from original to new
		_QueryMap = new int[OriginalNodes.length];
		Value unity = new ValueDouble(1.0);

		Preconditions Pre = new Preconditions(_OriginalNetwork);
		
		
		/// construct the transformed network
		// first count items
		_TransformedNetwork = bn.copy();
		int tCount = 0;
		int dCount = 0;
		int uCount = 0;
		for(int i = 0; i < OriginalNodes.length; i++)
		{
			if(OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
			{
				if(!OriginalNodes[i].hasEvidence())
				{
					dCount ++;
				}
			}
			if(OriginalNodes[i].getType() == BeliefNode.NODE_UTILITY)
			{
				uCount ++;
			}
		}
		// second, build buffers
		_DescisionNodes = new BeliefNode[dCount];
		_UtilityNodes = new BeliefNode[uCount];
		dCount = 0;
		uCount = 0;
		for(int i = 0; i < OriginalNodes.length; i++)
		{
			if (OriginalNodes[i].getType() == BeliefNode.NODE_CHANCE
					|| OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
			{
				_TransformedNetwork.getNodes()[tCount].setEvidence(OriginalNodes[i].getEvidence());
				_QueryMap[i] = tCount;
				
				if(OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
				{
					if(!OriginalNodes[i].hasEvidence())
					{
						_DescisionNodes[dCount] = _TransformedNetwork.getNodes()[tCount];
						dCount++;
					}
					CPF cpf = OriginalNodes[i].getCPF();
					int r = 0;
					while(r < cpf.size())
					{
						cpf.put(r, unity);
						r++;
					}
					_TransformedNetwork.getNodes()[tCount].setCPF(cpf);
				}
				tCount++;
			}
			else
			{
				_UtilityNodes[uCount] = OriginalNodes[i];
				uCount++;
				_TransformedNetwork.deleteBeliefNode(_TransformedNetwork.getNodes()[tCount]);
			}
		}
		if(dCount == 0)
		{
			inferenceMethod.run(_TransformedNetwork);
		}
		else
		{
			// collect the group information
			// this is the single bin
			double max = -1000000;
			CPF Decisions = new CPF(_DescisionNodes);
			int[] q = Decisions.realaddr2addr(0);
			int[] qBest = Decisions.realaddr2addr(0);
			boolean done = false;
			while(!done)
			{
				for(int i = 0; i < q.length; i++)
				{
					_DescisionNodes[i].setEvidence(new DiscreteEvidence(q[i]));
				}
				inferenceMethod.run(_TransformedNetwork);
				//
				Value U = evalUtility();
				if(U instanceof ValueDouble)
				{
					double V = ((ValueDouble)U).getValue();
					if(V > max)
					{
						max = V;
						for(int k = 0; k < q.length; k++)
							qBest[k] = q[k];
					}
				}
				done = Decisions.addOne(q);
			}
			for(int i = 0; i < q.length; i++)
			{
				_DescisionNodes[i].setEvidence(new DiscreteEvidence(qBest[i]));
				inferenceMethod.run(_TransformedNetwork);
			}
		}
	}
	public CPF queryMarginal(BeliefNode bnode)
	{
		if(bnode.getType() == BeliefNode.NODE_UTILITY)
		{
			Value V = evalUtilityOn(bnode);
			CPF X = new CPF(new BeliefNode[] { bnode });
			X.put(0,V);
			return X;
			//return bnode.getCPF();
		}
		else
		{
			int sQ = _QueryMap[bnode.loc()];
			BeliefNode Q = _TransformedNetwork.getNodes()[sQ];
			CPF Marginal = inferenceMethod.queryMarginal(Q); 
			return Marginal;
		}
		///return inferenceMethod.queryMarginal(bnode);
	}
	public int bestDescision(BeliefNode descision_node)
	{
		int sQ = _QueryMap[descision_node.loc()];
		BeliefNode Q = _TransformedNetwork.getNodes()[sQ];
		if(Q.hasEvidence())
			return ((DiscreteEvidence) Q.getEvidence()).getDirectValue();
		return 0;
	}
	public Value utility(BeliefNode utility_node)
	{
		return null;
	}
}

/*
int uCount = 0;
int dCount = 0;
int tCount = 0;
_QueryMap = new int[OriginalNodes.length];
for (int i = 0; i < OriginalNodes.length; i++)
{
	if (OriginalNodes[i].getType() == BeliefNode.NODE_CHANCE
			|| OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
	{
		tCount++;
		if (OriginalNodes[i].getType() == BeliefNode.NODE_DECISION) dCount++;
	}
	else
	{
		uCount++;
	}
}
_ChanceAndDescNodes = new BeliefNode[tCount];
_UtilityNodes = new BeliefNode[uCount];
_DescisionNodes = new BeliefNode[dCount];
// add nodes
tCount = 0;
uCount = 0;
dCount = 0;
for (int i = 0; i < OriginalNodes.length; i++)
{
	if (OriginalNodes[i].getType() == BeliefNode.NODE_CHANCE
			|| OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
	{
		_ChanceAndDescNodes[tCount] = new BeliefNode(OriginalNodes[i].getName(),
				                                     OriginalNodes[i].getDomain());
		_TransformedNetwork.addBeliefNode(_ChanceAndDescNodes[tCount]);
		
		_QueryMap[OriginalNodes[i].loc()] = tCount; 
		if (OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
		{
			// to force it as an evidence node?
			_DescisionNodes[dCount] = OriginalNodes[i];
			dCount++;
		}
		else
		{
			_ChanceAndDescNodes[tCount].setEvidence(OriginalNodes[i].getEvidence());
		}
		tCount++;
	}
	else
	{
		_UtilityNodes[uCount] = OriginalNodes[i];
		uCount++;
	}
}
// connect nodes
for (int i = 0; i < OriginalNodes.length; i++)
{
	BeliefNode[] Children = bn.getChildren(OriginalNodes[i]);
	if (OriginalNodes[i].getType() == BeliefNode.NODE_CHANCE
			|| OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
	{
		for (int k = 0; k < Children.length; k++)
		{
			BeliefNode C = Children[k];
			if (C.getType() == BeliefNode.NODE_CHANCE || C.getType() == BeliefNode.NODE_DECISION)
			{
				_TransformedNetwork.connect(_ChanceAndDescNodes[_QueryMap[OriginalNodes[i].loc()]],
						_ChanceAndDescNodes[_QueryMap[C.loc()]]);
			}
		}
	}
}
	

// upload cpfs
for (int i = 0; i < OriginalNodes.length; i++)
{
	int sQ = _QueryMap[i];
	if (OriginalNodes[i].getType() == BeliefNode.NODE_CHANCE)
	{
		//String original = Options.getString(_ChanceAndDescNodes[sQ].getCPF());
		_ChanceAndDescNodes[sQ].setCPF(OriginalNodes[i].getCPF().hardcopy());
		//String next = Options.getString(_ChanceAndDescNodes[sQ].getCPF());
		//System.out.println("CPF: " + original + "\n::>\n" + next);
	}
}
*/
/*
BeliefNode[] newNodes = _TransformedNetwork.getNodes();
for(int i = 0; i < newNodes.length; i++)
{
	System.out.println(" " + newNodes[i].getName());
	BeliefNode[] children = _TransformedNetwork.getParents(newNodes[i]);
	for(int j = 0; j < children.length; j++)
		System.out.println(" -> " + children[j].getName());
	System.out.println("    @@ " + Options.getString(newNodes[i].getCPF()));
}
*/
/*
Elimbel E = new Elimbel();
E.run(_TransformedNetwork.copy());
for(int i = 0; i < _TransformedNetwork.getNodes().length; i++)
{
	//CPF X = E.queryMarginal(_TransformedNetwork.getNodes()[i]);
	CPF X =_TransformedNetwork.getNodes()[i].getCPF();
	String cpf = Options.getString(X);
	System.out.println("" + _TransformedNetwork.getNodes()[i].loc() + ":::cpf for " + cpf + "\n");
}
*/

//System.out.println("constructed");
//inferenceMethod.run(_TransformedNetwork);

/*
 else if(OriginalNodes[i].getType() == BeliefNode.NODE_DECISION)
 {
 CPF X = _ChanceAndDescNodes[sQ].getCPF();
 for(int k = 0; k < X.size(); k++)
 X.put(k, unity);
 _ChanceAndDescNodes[sQ].setCPF(X);
 }
 */

/*
inferenceMethod.run(_TransformedNetwork);
for(int i = 0; i < _TransformedNetwork.getNodes().length; i++)
{
	//CPF X = inferenceMethod.queryMarginal(_TransformedNetwork.getNodes()[i]);
	CPF X =_TransformedNetwork.getNodes()[i].getCPF();
	String cpf = Options.getString(X);
	System.out.println(":::marginal for " + cpf + "\n");
}
*/

// we factor out the structure;
