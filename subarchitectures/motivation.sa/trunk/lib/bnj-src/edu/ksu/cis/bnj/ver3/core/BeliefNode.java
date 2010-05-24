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
import edu.ksu.cis.util.graph.core.Vertex;
/*!
 * \file BeliefNode.java
 * \author Jeffrey M. Barber
 */
public class BeliefNode
{
	public static final int NODE_CHANCE = 0;
	public static final int NODE_DECISION = 1;
	public static final int NODE_UTILITY = 2;
	
	private Vertex		_Owner;
	private CPF			_ConditionalProbabilityFunction;
	private Domain		_Domain;
	private String		_Name;
	private Evidence	_Evidence;
	private int			_Type;
	/*!
	 * Construct a node from a name and a domain
	 * \param[in] name 	the name of the node
	 * \param[in] d		the Domain of the node (i.e. Discrete / Continuous)
	 */
	public BeliefNode(String name, Domain d)
	{
		_Owner = null;
		_Name = name;
		_Domain = d;
		BeliefNode[] init = new BeliefNode[1];
		init[0] = this;
		_ConditionalProbabilityFunction = new CPF(init);
		_Evidence = null;
		_Type = BeliefNode.NODE_CHANCE;
	}
	/*! set the type of this node
	 * \param[in] type - the new type of this node, either it be CHANCE, DECISION, UTILITY
	 */
	public void setType(int type)
	{
		if(0 <= type && type <= 2)
		{
			_Type = type;
		}
	}
	/*! get the node type
	 * \return the type of this node, be it either CHANCE, DECISION, UTILITY
	 */
	public int getType()
	{
		return _Type;
	}
	/*! Set the domain of the node
	 * \note
	 * \par
	 * 		DO NOT USE unless you plan to change it back
	 * \param[in] D		the new domain, but the CPF associated to this are now broken
	 */
	public void setDomain(Domain D)
	{
		_Domain = D;
	}
	/*! Set the name of the node
	 * \note
	 * \par(for visual/reporting only, not use except for that)
	 * \param[in] name the new name
	 */
	public void setName(String name)
	{
		_Name = name;
	}
	/*! Set the evidence of this node
	 * \param[in] e 	the evidence value
	 */
	public void setEvidence(Evidence e)
	{
		_Evidence = e;
	}
	/*! Get the evidence of this node
	 * \return the evidence associate to this node (no evidence -> null)
	 */
	public Evidence getEvidence()
	{
		return _Evidence;
	}
	/*! Does this node have evidence?
	 * \return the answer to this question
	 */
	public boolean hasEvidence()
	{
		return _Evidence != null;
	}
	/*! Get the name of the node
	 * \note
	 * \par(for visual/reporting only, not use except for that)
	 * \return the name of the node
	 */
	public String getName()
	{
		return _Name;
	}
	/*! Get the CPF of this node
	 * \return the cpf of this node
	 */
	public CPF getCPF()
	{
		return _ConditionalProbabilityFunction;
	}
	/*! Set the CPF of this node
	 * \note
	 * \par
	 * 		DO NOT USE unless you plan to update the network
	 * \param[in] cpf 	the new cpf
	 */
	public void setCPF(CPF cpf)
	{
		_ConditionalProbabilityFunction = cpf;
	}
	/*! Get the domain of the cpf
	 * \return the domain
	 */
	public Domain getDomain()
	{
		return _Domain;
	}
	/*! set the Owner (a Vertex)
	 * \note
	 * \par
	 * 		DO NOT USE!!!!!!!!!!
	 * \param v
	 */
	public void setOwner(Vertex v)
	{
		_Owner = v;
	}
	/*! Get the Owner (a Vertex)
	 * \note
	 * \par
	 * 		TRY NOT TO USE THIS!!!
	 * \return the owner vertex
	 */
	public Vertex getOwner()
	{
		return _Owner;
	}
	/*!
	 * The optimization requirement
	 * \return the location index
	 */
	public int loc()
	{
		return _Owner.loc();
	}
	/*! Query a value from this node
	 * \param[in] query the query to the CPF or Evidence
	 * \return the value or the sum of values or the evidence
	 */
	public Value query(int[] query)
	{
		if (_Evidence != null)
		{
			return _Evidence.getEvidenceValue(query[0]);
		}
		return getCPF().get(query);
	}
	/*! Set the node's position
	 * \note
	 * \par(for visual/reporting only, not use except for that)
	 * @param[in] x 	the x component
	 * @param[in] y		the y component
	 */
	public void setPosition(int x, int y)
	{
		if (_Owner != null)
		{
			_Owner.translate(x - _Owner.getx(), y - _Owner.gety());
		}
	}
}