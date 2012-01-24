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
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.lazy.ConformalProduct;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.visualization.VisualizationController;
import edu.ksu.cis.util.graph.visualization.operators.CodePageSelectLine;
import edu.ksu.cis.util.graph.visualization.operators.CodePageUpdateEnvironment;
import edu.ksu.cis.util.graph.visualization.operators.Delay;
import edu.ksu.cis.util.graph.visualization.operators.VertexColor;
/*!
 * \file Clique.java
 * \author Jeffrey M. Barber
 */
public class Clique
{
	protected CPF			_CPF;
	protected LinkedList	_CliqueNodes	= new LinkedList();
	protected BeliefNode[]	_S				= null;
	protected BeliefNode[]	_Base			= null;
	protected CPF[]			_LambdaMessages;
	protected int			_ReportedChild	= 0;
	protected Vertex[]		_Children;
	protected Vertex[]		_Parents;
	protected Vertex		_Owner;
	protected VisualizationController _VC = null;
	/*! get the Clique's base nodes
	 * \return
	 */
	BeliefNode[] getBaseNodes()
	{
		return _Base;
	}
	/*! let the Clique know its parents and children for whom to message pass with
	 * \param[in] parents parents of the clique (other cliques)
	 * \param[in] children children of the clique (other cliques)
	 * \param[in] me the vertex that I am in
	 */
	void setupConnectivity(Vertex[] parents, Vertex[] children, Vertex me)
	{
		_Parents = parents;
		_Children = children;
		_LambdaMessages = new CPF[_Children.length];
		_Owner = me;
		/*
		String B = "{";
		for(int k = 0; k < _Base.length; k++)
		{
			B = (k==0 ? "{" : B + ",") + _Base[k].getName();
		}
		B+="}";
		me.setName(me.getName() + "\n" + B);
		*/
	}
	
	/*! Set the visualization Controller
	 * \param[in] VC The controller
	 */
	public void setVC(VisualizationController VC)
	{
		_VC = VC;
	}
	
	/*! Construct a Clique from a set of notes, a set of base nodes, a seperator set, an the original beliefnodes
	 * \param[in] nodes the set of nodes
	 * \param[in] basenodes the set of base nodes (for marginalization)
	 * \param[in] S the seperator set
	 * \param[in] originalbnodes the original bodies (for mapping)
	 */
	Clique(HashSet nodes, HashSet basenodes, HashSet S, BeliefNode[] originalbnodes)
	{
		if (nodes.size() == 0)
		{
			throw new RuntimeException("Something's wrong: Empty clique");
		}
		BeliefNode[] product = new BeliefNode[nodes.size()];
		int k = 0;
		for (Iterator i = nodes.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			product[k] = originalbnodes[v.loc()];
			_CliqueNodes.add(product[k]);
			k++;
		}
		_S = new BeliefNode[S.size()];
		k = 0;
		for (Iterator i = S.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			_S[k] = originalbnodes[v.loc()];
			k++;
		}
		k = 0;
		_Base = new BeliefNode[basenodes.size()];
		for (Iterator i = basenodes.iterator(); i.hasNext();)
		{
			Vertex v = (Vertex) i.next();
			_Base[k] = originalbnodes[v.loc()];
			k++;
		}
//		_CPF = new CPF(product,false);
		//int N = _CPF.size();
		//int[] query = _CPF.realaddr2addr(0);
		ConformalProduct CP = new ConformalProduct(product,_Base);
		_CPF = CP;
//		iterate();
	}
	/*! filter out nodes that have evidence
	 * \param[in] evNodes the list of nodes containing evidence
	 */
	void filterEvidenceNodes(LinkedList evNodes)
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
	/*! Begin the Lambda Propigation, calculates local lambda, divides by it, sends it upstream
	 */
	void lambdaPropigation()
	{
		boolean _vis = false;
		if(_VC != null)
		{
			_vis = true;
			_VC.pushAndApplyOperator(new VertexColor(_Owner, 14));
			_VC.pushAndApplyOperator(new Delay("delay_setlambdapropigation", 250));
			_VC.pushAndApplyOperator(new CodePageSelectLine(1));
			_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("C",_Owner.getName()));

			//_VC.pushAndApplyOperator(new CodePageSelectLine(0));
		}
		CPF lambda = null;
		if (_S.length > 0)
		{
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(2));
			lambda = _CPF.extract(_S);
			if(_vis)
			{
				String lMsg = "";
				for(int i = 0; i < lambda.getDomainProduct().length; i++)
				{
					lMsg = ((i==0) ? "" : lMsg+",") + lambda.getDomainProduct()[i].getName(); 
				}
				_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("lambda",lMsg));
			}
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(3));
			_CPF = CPF.divide(_CPF,lambda);
		}
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(4));
		if (_Parents.length == 1)
		{
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(5));
			((Clique) (_Parents[0].getObject())).setLambdaMessage(lambda);
		}
	}
	/*! Helper for calculating a child's pi value
	 * \param[in] p the parent's cpf
	 * @return return the extract pi value
	 */
	CPF getpi(CPF p)
	{
		if (_S.length > 0)
		{
			return p.extract(_S);
		}
		else
		{
			return null;
		}
	}
	/*! Pi Propigation, sends down the pipe pi messages 
	 */
	void piPropigation()
	{
		boolean _vis = false;
		if(_VC != null)
		{
			_vis = true;
			_VC.pushAndApplyOperator(new VertexColor(_Owner, 16));
			_VC.pushAndApplyOperator(new Delay("delay_pipropigation", 250));
			_VC.pushAndApplyOperator(new CodePageSelectLine(16));
			_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("C",_Owner.getName()));
		}		
		for (int i = 0; i < _Children.length; i++)
		{
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(17));
			Clique C = (Clique) _Children[i].getObject();
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(18));
			CPF pi = C.getpi(_CPF);
			if(_vis)
			{
				String pMsg = "";
				for(int j = 0; j < pi.getDomainProduct().length; j++)
				{
					pMsg = ((i==0) ? "" : pMsg+",") + pi.getDomainProduct()[j].getName(); 
				}
				_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("pi",pMsg));
			}

			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(19));
			C.setPiMessage(pi);
		}
	}
	/*! Set Lambda Message, event for incoming lambda messages, waits for all children to report.
	 * \param[in] lambda incoming lambda
	 */
	void setLambdaMessage(CPF lambda)
	{
		boolean _vis = false;
		if(_VC != null)
		{
			_vis = true;
			_VC.pushAndApplyOperator(new VertexColor(_Owner, 15));
			_VC.pushAndApplyOperator(new Delay("delay_setlambdamessage", 250));
			_VC.pushAndApplyOperator(new CodePageSelectLine(7));
			_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("C",_Owner.getName()));
		}
		_ReportedChild++;
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(8));
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(9));
		if (lambda != null) _LambdaMessages[_ReportedChild - 1] = lambda;
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(10));
		if (_ReportedChild < _Children.length) return;
		for (int i = 0; i < _ReportedChild; i++)
		{
			if (_LambdaMessages[i] != null)
			{
				if(_vis)
				{
					String lMsg = "";
					for(int j = 0; j < _LambdaMessages[i].getDomainProduct().length; j++)
					{
						lMsg = ((i==0) ? "" : lMsg+",") + _LambdaMessages[i].getDomainProduct()[j].getName(); 
					}
					_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("LI",lMsg));
				}

				if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(11));
				_CPF = CPF.multiply(_CPF,_LambdaMessages[i]);
			}
		}
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(12));
		lambdaPropigation();
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(13));
		if (_Parents.length == 0)
		{
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(14));
			piPropigation();
		}
	}
	/*! get this clique's CPF
	 * @return the CPF
	 */
	public CPF getCPF()
	{
		return _CPF;
	}
	/*! Incoming Pi Message, multiply and propigate down the stream
	 * \param[in] pi
	 */
	void setPiMessage(CPF pi)
	{
		boolean _vis = false;
		if(_VC != null)
		{
			_vis = true;
			_VC.pushAndApplyOperator(new VertexColor(_Owner, 17));
			_VC.pushAndApplyOperator(new Delay("delay_setpimessage", 250));
			_VC.pushAndApplyOperator(new CodePageSelectLine(21));
			_VC.pushAndApplyOperator(new CodePageUpdateEnvironment("C",_Owner.getName()));
		}
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(22));
		if (pi != null)
		{
			_CPF = CPF.multiply(_CPF,pi);
			//_CPF.multiply(pi);
		}
		if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(23));
		if (_Children.length > 0)
		{
			if(_vis) _VC.pushAndApplyOperator(new CodePageSelectLine(24));
			piPropigation();
		}
	}
	/*! Construct this node's CPF from all the base nodes
	 */
	void iterate()
	{
		/*
		for (int i = 0; i < N; i++)
		{
			Value result = CP.get(query);
			_CPF.put(i, result);
			_CPF.addOne(query);
		}
		*/
		/*
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
		if(_VC != null)
		{
			_VC.pushAndApplyOperator(new VertexColor(_Owner, 0));
		}
		*/
	}
}