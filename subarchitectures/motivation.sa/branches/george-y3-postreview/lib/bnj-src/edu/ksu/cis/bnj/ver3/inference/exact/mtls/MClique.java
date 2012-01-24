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
 */package edu.ksu.cis.bnj.ver3.inference.exact.mtls;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.Field;
import edu.ksu.cis.bnj.ver3.core.values.ValueUnity;
import edu.ksu.cis.util.graph.core.Vertex;
/*!
 * \file MClique.java
 * \author Jeffrey M. Barber
 */
public class MClique
{
	protected CPF			_CPF;
	protected LinkedList	_CliqueNodes	= new LinkedList();
	protected BeliefNode[]	_S				= null;
	protected BeliefNode[]	_Base			= null;
	protected BeliefNode[]	_Product;
	protected CPF[]			_LambdaMessages;
	protected int			_ReportedChild	= 0;
	protected Vertex[]		_Children;
	protected Vertex[]		_Parents;
	protected int			_Processor;
	/*! get the assign'd processor
	 * \return
	 */
	public int getProcessor()
	{
		return _Processor;
	}
	/*! assign this to a processor
	 * \param[in] P the processor ident
	 */
	public void assignProcessor(int P)
	{
		_Processor = P;
	}
	/*! get the cliques's cpf
	 * \return return the cpf
	 */
	public CPF getCPF()
	{
		return _CPF;
	}
	/*! get the base nodes
	 * \return the base nodes
	 */
	public BeliefNode[] getBaseNodes()
	{
		return _Base;
	}
	/*! set the connectivity of this clique 
	 * \param[in] parents the parents of this clique
	 * \param[in] children the children of this clique
	 */
	public void setupConnectivity(Vertex[] parents, Vertex[] children)
	{
		_Parents = parents;
		_Children = children;
		_LambdaMessages = new CPF[_Children.length];
	}
	/*! Construct a Clique from a set of notes, a set of base nodes, a seperator set, an the original beliefnodes
	 * \param[in] nodes the set of nodes
	 * \param[in] basenodes the set of base nodes (for marginalization)
	 * \param[in] S the seperator set
	 * \param[in] originalbnodes the original bodies (for mapping)
	 */
	public MClique(HashSet nodes, HashSet basenodes, HashSet S, BeliefNode[] originalbnodes)
	{
		_Processor = -1;
		// error msg
		if (nodes.size() == 0)
		{
			throw new RuntimeException("Something's wrong: Empty clique");
		}
		// create product domain
		_Product = new BeliefNode[nodes.size()];
		// build product domain
		int k = 0;
		for (Iterator i = nodes.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			_Product[k] = originalbnodes[v.loc()];
			_CliqueNodes.add(_Product[k]);
			k++;
		}
		// build the S set
		_S = new BeliefNode[S.size()];
		k = 0;
		for (Iterator i = S.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			_S[k] = originalbnodes[v.loc()];
			k++;
		}
		// build the base nodes
		_Base = new BeliefNode[basenodes.size()];
		k = 0;
		for (Iterator i = basenodes.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			_Base[k] = originalbnodes[v.loc()];
			k++;
		}
	}
	/*! incoming message, build the CPF
	 */
	public void localBuildCPF()
	{
		_CPF = new CPF(_Product);
		int N = _CPF.size();
		int[] query = _CPF.realaddr2addr(0);
		Value unity = new ValueUnity();
		int[][] baseQueryProjectionCache = new int[_Base.length][];
		int[][] baseQueryAddressCache = new int[_Base.length][];
		int[] _SubsetCache = new int[_Base.length];
		BeliefNode[] cdp = _CPF.getDomainProduct();
		for (int j = 0; j < _Base.length; j++)
		{
			baseQueryProjectionCache[j] = CPF.getProjectionMapping(_CPF.getDomainProduct(), _Base[j].getCPF()
					.getDomainProduct());
			baseQueryAddressCache[j] = _Base[j].getCPF().realaddr2addr(0);
			for (int k = 0; k < cdp.length; k++)
			{
				if (cdp[k] == _Base[j])
				{
					_SubsetCache[j] = k;
				}
			}
		}
		for (int i = 0; i < N; i++)
		{
			Value result = unity;
			for (int j = 0; j < _Base.length; j++)
			{
				CPF.applyProjectionMapping(query, baseQueryProjectionCache[j], baseQueryAddressCache[j]);
				Value temp;
				if (_Base[j].hasEvidence()
						&& _Base[j].getEvidence().getEvidenceValue(query[_SubsetCache[j]]) instanceof ValueUnity)
				{
					temp = _Base[j].getCPF().get(baseQueryAddressCache[j]);
				}
				else
				{
					temp = _Base[j].query(baseQueryAddressCache[j]);
				}
				result = Field.mult(result, temp);
			}
			_CPF.put(i, result);
			_CPF.addOne(query);
		}
	}
	/*! incoming message, filter local evidence
	 * \param evNodes the evidence nodes
	 */
	public void localFilterEvidence(LinkedList evNodes)
	{
		if (evNodes != null)
		{
			LinkedList releventNodes = new LinkedList();
			releventNodes.addAll(evNodes);
			releventNodes.retainAll(_CliqueNodes);
			if (releventNodes.size() > 0)
			{
				LinkedList S = new LinkedList();
				for (int i = 0; i < _S.length; i++)
				{
					S.add(_S[i]);
				}
				S.removeAll(releventNodes);
				int k = 0;
				_S = new BeliefNode[S.size()];
				for (Iterator i = S.iterator(); i.hasNext();)
				{
					BeliefNode bnode = (BeliefNode) i.next();
					_S[k] = bnode;
					k++;
				}
				BeliefNode[] relNodes = new BeliefNode[releventNodes.size()];
				k = 0;
				for (Iterator j = releventNodes.iterator(); j.hasNext();)
				{
					BeliefNode bnode = (BeliefNode) j.next();
					relNodes[k] = bnode;
					k++;
				}
				_CPF.zeroExceptForNodeEvidence(relNodes);
			}
		}
	}
	/*! incoming message, lambda propigation
	 * \param[in] MC the message coordinator
	 */
	public void localLambdaPropigation(MessageCoordinator MC)
	{
		CPF lambda = null;
		if (_S.length > 0)
		{
			lambda = _CPF.extract(_S);
			_CPF = CPF.divide(_CPF,lambda);
			//_CPF.divide(lambda);
		}
		if (_Parents.length == 1)
		{
			MClique P = ((MClique) (_Parents[0].getObject()));
			MC.queueSetLambdaMessage(P, lambda);
		}
	}
	/*! incoming message, set lambda message
	 * \param[in] lambda the incoming lambda message 
	 * \param[in] MC the coordinator of messages
	 */
	public void localSetLambdaMessage(CPF lambda, MessageCoordinator MC)
	{
		_ReportedChild++;
		if (lambda != null) _LambdaMessages[_ReportedChild - 1] = lambda;
		if (_ReportedChild < _Children.length) return;
		for (int i = 0; i < _ReportedChild; i++)
		{
			if (_LambdaMessages[i] != null)
			{
				_CPF = CPF.multiply(_CPF,_LambdaMessages[i]);
				//_CPF.multiply(_LambdaMessages[i]);
			}
		}
		MC.queueLambdaPropigation(this);
		if (_Parents.length == 0)
		{
			MC.queuePiPropigation(this);
		}
	}
	/*! incoming Message, Pi Propigation
	 * \param[in] MC the coordinator of messages
	 */
	public void localPiPropigation(MessageCoordinator MC)
	{
		MC.queueBuildMarginal(this);
		for (int i = 0; i < _Children.length; i++)
		{
			MClique C = (MClique) _Children[i].getObject();
			MC.queueSetPiMessage(C, _CPF);
		}
	}
	/*! incoming message, Set Pi Message
	 * \param[in] msg
	 * \param[in] MC the coordinator of messages
	 */
	public void localSetPiMessage(CPF msg, MessageCoordinator MC)
	{
		CPF pi = null;
		if (_S.length > 0)
		{
			pi = msg.extract(_S);
			_CPF = CPF.multiply(_CPF,pi);
			//_CPF.multiply(pi);
		}
		MC.queuePiPropigation(this);
	}
}