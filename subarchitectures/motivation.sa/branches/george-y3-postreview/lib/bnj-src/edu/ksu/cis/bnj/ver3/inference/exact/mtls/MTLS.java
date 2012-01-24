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
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.util.graph.algorithms.BuildCliqueTree;
/*!
 * \file MTLS.java
 * \author Jeffrey M. Barber
 */
public class MTLS implements Inference, LSMarginalHook
{
	private CPF[]	marginals	= null;
	/*! Run the Inference
	 * \param[in] bn the Belief Network to run inference on
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::run(edu.ksu.cis.bnj.ver3.core.BeliefNetwork)
	 */
	public void run(BeliefNetwork bn)
	{
		BuildCliqueTree BCT = new BuildCliqueTree();
		BCT.setGraph(bn.getGraph());
		BCT.execute();
		MCliqueTree CT = new MCliqueTree(BCT, bn, this, 2);
	}
	/*! Run the Inference
	 * \param[in] bn the Belief Network to run inference on
	 * \param[in] NumberOfProcessors the Number of processors
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::run(edu.ksu.cis.bnj.ver3.core.BeliefNetwork)
	 */
	public void run(BeliefNetwork bn, int NumberOfProcessors)
	{
		BuildCliqueTree BCT = new BuildCliqueTree();
		BCT.setGraph(bn.getGraph());
		BCT.execute();
		MCliqueTree CT = new MCliqueTree(BCT, bn, this, NumberOfProcessors);
	}
	/*! Get the name of this Inference Algorithm
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::getName()
	 */
	public String getName()
	{
		return "LS Message Passing - Multi Threaded";
	}
	/*! Wait until the marginals arrive
	 */
	public void waitUntilRecieved()
	{
		while (marginals == null)
		{
			try
			{
				Thread.yield();
			}
			catch (Exception e)
			{}
		}
	}
	/*! The marginals have arrived
	 * \see edu.ksu.cis.bnj.ver3.inference.exact.mtls.LSMarginalHook::OnMarginals(edu.ksu.cis.bnj.ver3.core.CPF[])
	 */
	public void OnMarginals(CPF[] Marginals)
	{
		marginals = Marginals;
	}
	/*! Query for a Marginal
	 * \param[in] bnode the Belief Node to query the marginal of
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::queryMarginal()
	 */
	public CPF queryMarginal(BeliefNode bnode)
	{
		if (marginals == null) return null;
		return marginals[bnode.loc()];
	}
}