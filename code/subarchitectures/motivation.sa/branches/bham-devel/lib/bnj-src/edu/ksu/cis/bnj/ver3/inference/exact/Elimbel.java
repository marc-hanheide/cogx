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
 */package edu.ksu.cis.bnj.ver3.inference.exact;
import java.util.*;
import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.values.*;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.util.data.OrderedList;
import edu.ksu.cis.util.graph.algorithms.MaximumCardinalitySearch;
/*!
 * \file Elimbel.java
 * \author Jeffrey M. Barber
 */
public class Elimbel implements Inference
{
	private boolean			_HasRunBeenCalled;
	private BeliefNode[]	_Nodes;
	private CPF[]			_Lambda;
	private Set[]			_LambdaParameters;
	private Set[]			_LambdaTable;
	private OrderedList[]	_BaseNodes;
	private BeliefNetwork	_BeliefNet;
	// JMB: Caches for extra boost
	private int[][]			_BaseProjectionCache;
	private int[][]			_BaseQueryCache;
	private int[][]			_LambdaProjectionCache;
	private int[][]			_LambdaQueryCache;
	private int[]			_SubsetCache;
	/*! Construct Elimbel
	 */
	public Elimbel()
	{
		_HasRunBeenCalled = false;
	}
	/*! Get the name of this Inference Algorithm
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::getName()
	 */
	public String getName()
	{
		return "Variable Elimination (elimbel)";
	}
	/*! Query for a Marginal
	 * \param[in] bnode the Belief Node to query the marginal of
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::queryMarginal()
	 */
	public CPF queryMarginal(BeliefNode bnode)
	{
		if (!_HasRunBeenCalled) return null;
		BeliefNode[] marginal = new BeliefNode[1];
		marginal[0] = bnode;
		CPF result = _Lambda[bnode.loc()].extract(marginal);
		result.normalize();
		return result;
	}
	/*! Run the Inference
	 * \param[in] bnet the Belief Network to run inference on
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::run(edu.ksu.cis.bnj.ver3.core.BeliefNetwork)
	 */
	public void run(BeliefNetwork bnet)
	{
		boolean old = CPF._LazyEvalEnabled;
		CPF._LazyEvalEnabled = false;
		_BeliefNet = bnet;
		MaximumCardinalitySearch MCS = new MaximumCardinalitySearch();
		MCS.setGraph(bnet.getGraph());
		MCS.execute();
		int[] ordering = MCS.getAlpha();
		for (int i = 0; i < ordering.length; i++)
			ordering[i]--;
		_BeliefNet.applyOrder(ordering);
		// preprocessing and cache building
		_Nodes = _BeliefNet.getNodes();
		_Lambda = new CPF[_Nodes.length];
		_LambdaParameters = new Set[_Nodes.length];
		_LambdaTable = new Set[_Nodes.length];
		_BaseNodes = new OrderedList[_Nodes.length];
		_BaseProjectionCache = new int[_Nodes.length][];
		_BaseQueryCache = new int[_Nodes.length][];
		_SubsetCache = new int[_Nodes.length];
		_LambdaProjectionCache = new int[_Nodes.length][];
		_LambdaQueryCache = new int[_Nodes.length][];
		for (int i = 0; i < _Nodes.length; i++)
		{
			_LambdaParameters[i] = new HashSet();
			_LambdaTable[i] = new HashSet();
			_BaseNodes[i] = new OrderedList();
		}
		backwarePhase();
		forwardPhase();
		_HasRunBeenCalled = true;
		CPF._LazyEvalEnabled = old;
	}
	/*!
	 * Backward phase of variable elimination. This involves constructing
	 * buckets and building lambda functions (or, rather, lambda table)
	 */
	private void backwarePhase()
	{
		boolean hasSeen[] = new boolean[_Nodes.length];
		for (int i = _Nodes.length - 1; i >= 0; i--)
		{
			hasSeen[i] = false;
		}
		for (int i = _Nodes.length - 1; i >= 0; i--)
		{
			BeliefNode node = _Nodes[i];
			if (!hasSeen[node.loc()])
			{
				hasSeen[node.loc()] = true;
				_BaseNodes[i].add(node);
				_LambdaParameters[i].add(node);
				BeliefNode[] parents = _BeliefNet.getParents(node);
				/*
				 * Adding the parents to be included into the lambda parameter
				 */
				if (parents != null)
				{
					for (int j = 0; j < parents.length; j++)
					{
						BeliefNode parent = parents[j];
						_LambdaParameters[i].add(parent);
					}
				}
			}
			/*
			 * Looking for P(child | curNode, otherParents...) to be included in the
			 * current bucket
			 */
			BeliefNode[] children = _BeliefNet.getChildren(node);
			for (int j = 0; j < children.length; j++)
			{
				BeliefNode child = children[j];
				if (!hasSeen[child.loc()])
				{
					hasSeen[child.loc()] = true;
					_BaseNodes[i].add(child);
					_LambdaParameters[i].add(child);
					BeliefNode[] childParents = _BeliefNet.getParents(child);
					if (childParents != null)
					{
						if (childParents.length > 0) for (int k = 0; k < childParents.length; k++)
						{
							_LambdaParameters[i].add(childParents[k]);
						}
					}
				}
			}
			// Build the lambda
			BeliefNode[] lambdaSubset = new BeliefNode[_LambdaParameters[i].size()];
			int k = 0;
			for (Iterator j = _LambdaParameters[i].iterator(); j.hasNext();)
			{
				BeliefNode param = (BeliefNode) j.next();
				lambdaSubset[k] = param;
				k++;
			}
			buildLambda(i, lambdaSubset);
			if (i > 0)
			{
				// Find the highest bucket index that is lower than the current bucket
				int highestBucketIndex = -1;
				for (int j = 0; j < lambdaSubset.length; j++)
				{
					int idx = lambdaSubset[j].loc();
					if (idx < i && idx > highestBucketIndex)
					{
						highestBucketIndex = idx;
					}
				}
				// found the bucket, float the parameters up
				if (highestBucketIndex > -1)
				{
					_LambdaTable[highestBucketIndex].add(new Integer(i));
					for (int j = 0; j < lambdaSubset.length; j++)
					{
						if (lambdaSubset[j] != node) _LambdaParameters[highestBucketIndex].add(lambdaSubset[j]);
					}
				}
			}
		}
	}
	/*! build the lambda table for curIndex using the paramters from lambdaSubset
	 * \param[in] curIndex - which to build
	 * \param[in] lambdaSubset - paramters
	 */
	private void buildLambda(int curIndex, BeliefNode[] lambdaSubset)
	{
		_Lambda[curIndex] = new CPF(lambdaSubset,false);
		// build the projection for the base under this subset (gives about 10% boost including building cost)
		//for (Iterator j = _BaseNodes[curIndex].iterator(); j.hasNext();)
		for (int j = 0; j < _BaseNodes[curIndex].size(); j++)
		{
			BeliefNode base = (BeliefNode) _BaseNodes[curIndex].get(j);// j.next();
			_BaseProjectionCache[base.loc()] = CPF.getProjectionMapping(lambdaSubset, base.getCPF().getDomainProduct());
			_BaseQueryCache[base.loc()] = base.getCPF().realaddr2addr(0);
			for (int k = 0; k < lambdaSubset.length; k++)
			{
				if (lambdaSubset[k] == base)
				{
					_SubsetCache[base.loc()] = k;
				}
			}
		}
		// build the projection for the lambdas (gives about 10%)
		for (Iterator j = _LambdaTable[curIndex].iterator(); j.hasNext();)
		{
			int idx = ((Integer) j.next()).intValue();
			_LambdaProjectionCache[idx] = CPF.getProjectionMapping(lambdaSubset, _Lambda[idx].getDomainProduct());
			_LambdaQueryCache[idx] = _Lambda[idx].realaddr2addr(0);
		}
		// get the query variable for this lambda
		int[] query = _Lambda[curIndex].realaddr2addr(0);
		Value unity = new ValueUnity();
		for (int i = 0; i < _Lambda[curIndex].size(); i++)
		{
			// for every value in the CPF
			Value result = unity;
			for (int j = 0; j < _BaseNodes[curIndex].size(); j++)
			{
				// for every base node
				BeliefNode base = (BeliefNode) _BaseNodes[curIndex].get(j);// j.next();
				// use the projection to build the query in the the base's cpf
				CPF.applyProjectionMapping(query, _BaseProjectionCache[base.loc()], _BaseQueryCache[base.loc()]);
				// get the value from the cpf or evidence
				Value p;
				if (base.hasEvidence()
						&& base.getEvidence().getEvidenceValue(query[_SubsetCache[base.loc()]]) instanceof ValueUnity)
				{
					p = base.getCPF().get(_BaseQueryCache[base.loc()]);
				}
				else
				{
					//p = _BeliefNet.Query(base, _BaseQueryCache[base.loc()]);
					p = base.query(_BaseQueryCache[base.loc()]);
				}
				// multipy it out in the field
				result = Field.mult(result, p);
			}
			for (Iterator j = _LambdaTable[curIndex].iterator(); j.hasNext();)
			{
				int idx = ((Integer) j.next()).intValue();
				// for every index, use the project for compatible query
				CPF.applyProjectionMapping(query, _LambdaProjectionCache[idx], _LambdaQueryCache[idx]);
				Value p = _Lambda[idx].get(_LambdaQueryCache[idx]);
				// multiply out in the field
				result = Field.mult(result, p);
			}
			// put the result
			_Lambda[curIndex].put(i, result);
			// inrement the query variable
			_Lambda[curIndex].addOne(query);
		}
	}
	/*!
	 * Forward phase of variable elimination.
	 */
	private void forwardPhase()
	{
		for (int i = 0; i < _Nodes.length; i++)
		{
			// for every node
			for (Iterator j = _LambdaTable[i].iterator(); j.hasNext();)
			{
				// for every lambda
				int idx = ((Integer) j.next()).intValue();
				HashSet sepset = new HashSet();
				sepset.addAll(_LambdaParameters[idx]);
				sepset.retainAll(_LambdaParameters[i]);
				if (sepset.size() > 0)
				{
					BeliefNode[] nodesepset = new BeliefNode[sepset.size()];
					int org = 0;
					for (Iterator k = sepset.iterator(); k.hasNext();)
					{
						nodesepset[org] = (BeliefNode) k.next();
						org++;
					}
					// the divisor extract to make compatible
					CPF lambdaDivisor = _Lambda[idx].extract(nodesepset);
					_Lambda[idx] = CPF.divide(_Lambda[idx],lambdaDivisor);
					// the multiplier
					_Lambda[idx] = CPF.multiply(_Lambda[idx],_Lambda[i]);
				}
			}
		}
		_Nodes = null;
		_LambdaParameters = null;
		_LambdaTable = null;
		_BaseNodes = null;
		_BaseProjectionCache = null;
		_BaseQueryCache = null;
		_SubsetCache = null;
		_LambdaProjectionCache = null;
		_LambdaQueryCache = null;
	}
}