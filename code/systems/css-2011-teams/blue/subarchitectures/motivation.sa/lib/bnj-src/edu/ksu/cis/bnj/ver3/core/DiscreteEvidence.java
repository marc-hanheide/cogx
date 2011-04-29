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
import edu.ksu.cis.bnj.ver3.core.values.ValueUnity;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
/*!
 * \file DiscreteEvidence.java
 * \author Jeffrey M. Barber
 */
public class DiscreteEvidence implements Evidence
{
	private int	_Which;
	/*! Get the direct mapping
	 * \return the direct mapping of evidence
	 */
	public int getDirectValue()
	{
		return _Which;
	}
	/*! Construct a Discrete Evidence value from 
	 * @param[in] Value - direct mapping of evidence
	 */
	public DiscreteEvidence(int Value)
	{
		_Which = Value;
	}
	/*! Helper Construct
	 * \param[in] D		 		the discrete domain to search in
	 * \param[in] Outcome	    the outcome to find
	 */
	public DiscreteEvidence(Discrete D, String Outcome)
	{
		_Which = D.findName(Outcome);
	}
	/*! Is this evidence correct for this domain?
	 * \see edu.ksu.cis.bnj.ver3.core.Evidence::inDomain()
	 * \param[in] D		the domain to check against
	 * \return the result ot the question [true/false]
	 */
	public boolean inDomain(Domain D)
	{
		return (D instanceof Discrete);
	}
	/*! Get the value in the domain for this evidence
	 * \see edu.ksu.cis.bnj.ver3.core.Evidence#getEvidenceValue()
	 * \return the value of the evivence
	 */
	public Value getEvidenceValue(int q)
	{
		if (q == _Which)
		{
			return ValueUnity.SingletonUnity;
		}
		return ValueZero.SingletonZero;
	}
	/*! Get the associated name of the value
	 * \see edu.ksu.cis.bnj.ver3.core.Evidence#getName()
	 * \return the name
	 */	
	public String getName(Domain D)
	{
		if(inDomain(D))
		{
			return ((Discrete)D).getName(_Which);
		}
		return null;
	}
	
}