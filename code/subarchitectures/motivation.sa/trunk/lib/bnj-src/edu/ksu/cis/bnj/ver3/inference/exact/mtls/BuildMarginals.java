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
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
/*!
 * \file BuildMarginals.java
 * \author Jeffrey M. Barber
 */
public class BuildMarginals extends Message
{
	private MessageCoordinator	_MessageCoordinator;
	private CPF[]				_Marginals;
	/*! Build Marginals
	 * \param[in] C the clique
	 * \param[out] MarginalPad the output reciever for the marginaliation
	 */
	public BuildMarginals(MClique C, CPF[] MarginalPad)
	{
		_Clique = C;
		_Marginals = MarginalPad;
	}
	/*! Run this Message
	 * \see edu.ksu.cis.bnj.ver3.inference.exact.mtls.Message::run()
	 */
	public void run()
	{
		BeliefNode[] M = _Clique.getBaseNodes();
		for (int j = 0; j < M.length; j++)
		{
			BeliefNode[] S = new BeliefNode[1];
			S[0] = M[j];
			_Marginals[M[j].loc()] = _Clique.getCPF().extract(S);
			_Marginals[M[j].loc()].normalize();
		}
	}
}