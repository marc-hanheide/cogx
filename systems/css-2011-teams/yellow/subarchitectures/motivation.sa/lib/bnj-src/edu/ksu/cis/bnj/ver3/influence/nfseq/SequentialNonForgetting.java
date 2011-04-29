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
 */package edu.ksu.cis.bnj.ver3.influence.nfseq;

import java.io.FileOutputStream;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueUnity;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
import edu.ksu.cis.bnj.ver3.inference.exact.LSonline;
import edu.ksu.cis.bnj.ver3.influence.Solver;
import edu.ksu.cis.bnj.ver3.influence.simple.SimpleSolve;
import edu.ksu.cis.bnj.ver3.plugin.IOPlugInLoader;
import edu.ksu.cis.bnj.ver3.streams.Exporter;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Writer;

// ------------------------
// STATUS: EXPERIMENTAL
// --- NOT PRODUCTION LEVEL

/*!
 * \file SequentialNonForgetting.java
 * \author Jeffrey M. Barber
 */
public class SequentialNonForgetting implements Solver
{
	private LSonline inference;
	private BeliefNetwork _TransformedNetwork;
	private boolean _Passed;
	private SimpleSolve _SolverAlternate;
	
	public SequentialNonForgetting()
	{
		inference = new LSonline();
	}
	/*! What is the name of this influence diagram solver
	 * \return the name
	 */
	public String getName()
	{
		return "Sequential Non Forgetting Solver";
	}
	
	/*! Solve this network
	 * 
	 * \param[in] bn the Bayesian Belief Network
	 */
	public void solve(BeliefNetwork bn)
	{
		_SolverAlternate = new SimpleSolve();

		Preconditions Pre = new Preconditions(bn);
		if(!Pre.passed || Pre._NumberOfDecisions <= 1)
		{
			_SolverAlternate.solve(bn);
			_Passed = false;
			return;
		}
		_Passed = true;

		BeliefNode[] _OriginalNodes = bn.getNodes();

		_TransformedNetwork = bn.copy();
		
		BeliefNode[] _DecisionNodes = new BeliefNode[Pre._NumberOfDecisions];
		int numDecisions = 0;
		
		ValueDouble unity = new ValueDouble(1.0);
		BeliefNode[] nodes = _TransformedNetwork.getNodes(); 
		for(int i = 0; i < nodes.length; i++)
		{
			BeliefNode X = nodes[Pre.order[i]];
			if(X.getType() == BeliefNode.NODE_DECISION)
			{
				// fill the cpf with ones
				CPF cpf = X.getCPF();
				for(int j = 0; j < cpf.size(); j++)
				{
					cpf.put(j, unity);
				}
				cpf.normalizeByDomain();
				X.setCPF(cpf);
				X.setType(BeliefNode.NODE_CHANCE);
				
				// if has evidence, copy
				
				if(_OriginalNodes[X.loc()].hasEvidence())
				{
					X.setEvidence(_OriginalNodes[X.loc()].getEvidence());
				}
				else
				{
					_DecisionNodes[numDecisions] = X;
					numDecisions++;
				}
			}
			else  if(X.getType() == BeliefNode.NODE_UTILITY)
			{
				double max = 0;
				CPF old = X.getCPF();
				BeliefNode[] par = X.getCPF().getDomainProduct();
				X.setDomain(new Discrete(new String[] {"t", "f"}));
				par[0] = X;
				CPF dom = new CPF(par);
				int k;
				for(k = 0; k < old.size(); k++)
				{
					ValueDouble d = (ValueDouble) old.get(k);
					if(d.getValue() > max)
						max = d.getValue();
				}
				for(k = 0; k < old.size(); k++)
				{
					ValueDouble d = (ValueDouble) old.get(k);
					double tV = d.getValue() / max;
					double fV = 1.0 - d.getValue() / max;
					dom.put(k             , new ValueDouble(tV));
					dom.put(k + old.size(), new ValueDouble(fV));
				}
				X.setCPF(dom);
				X.setEvidence(new DiscreteEvidence(0));
				X.setType(BeliefNode.NODE_CHANCE);
			}
			else
			{
				X.setEvidence(_OriginalNodes[X.loc()].getEvidence());
				// copy the evidence over
			}
		}

		for(int k = _DecisionNodes.length - 1; k >= 0; k--)
		{
			//_DecisionNodes[k]
			inference.run(_TransformedNetwork);
			CPF marginal = inference.queryMarginal(_DecisionNodes[k]);
			CPF cpf = _DecisionNodes[k].getCPF();
			int div = cpf.size() / marginal.size();
			for(int p = 0; p < marginal.size(); p++)
			{
				for(int p2 = 0; p2 < div; p2++)
				{
					int r = p * div + p2;
					cpf.put(r, marginal.get(p));
				}
			}
		}
		
		for(int k = _DecisionNodes.length - 1; k >= 0; k--)
		{
			CPF marginal = inference.queryMarginal(_DecisionNodes[k]);
			double max = 0;
			int kMax = 0;
			for(int i = 0; i < marginal.size(); i++)
			{
				ValueDouble d = (ValueDouble) marginal.get(i);
				if(d.getValue() > max)
				{
					max = d.getValue();
					kMax = i;
				}
			}
			_OriginalNodes[_DecisionNodes[k].loc()].setEvidence(new DiscreteEvidence(kMax));
//			_DecisionNodes[k].setEvidence( new DiscreteEvidence(kMax));
		}
		
		// marginalize after all decisions are made
		_SolverAlternate.solve(bn);

		// erase the decisions we made
		for(int k = _DecisionNodes.length - 1; k >= 0; k--)
		{
			_OriginalNodes[_DecisionNodes[k].loc()].setEvidence(null);
//			_DecisionNodes[k].setEvidence( new DiscreteEvidence(kMax));
		}
		
		/*
		try
		{
		IOPlugInLoader pil = IOPlugInLoader.getInstance();
		Exporter EXP = pil.GetExportersByExt(pil.GetExt(".xml"));
		EXP.save(new FileOutputStream("test.xml"));
		OmniFormatV1_Writer.Write( _TransformedNetwork , EXP.getStream1());
		} catch (Exception e)
		{
		}
		
		*/
		

		//for each decision
			// make uniform decision probability
		// for each utility
		    // transform from {v} to {t,f} and normalize
		// set evidence for all utility values to t
		
		
		// from last decision to current
			// run inference
		    // copy current decision node marginal into decision's cpf over all values
		// look at all decision's marginals, for each marginal
		
	}

	/*! After solving, query a marginal for a node
	 * 
	 * \param[in] bnode 	A Belief Node we want to query
	 * \return the cpf that contains the marginal
	 */
	public CPF queryMarginal(BeliefNode bnode)
	{
		return _SolverAlternate.queryMarginal(bnode);

		/*
		if(!_Passed)
		{
			return _SolverAlternate.queryMarginal(bnode);
		}
		if(bnode.getType() == BeliefNode.NODE_DECISION)
		{
			CPF X = new CPF(new BeliefNode[] { bnode });
			DiscreteEvidence E = (DiscreteEvidence) _TransformedNetwork.getNodes()[bnode.loc()].getEvidence();
			for(int i = 0; i < X.size(); i++)
				X.put(i, new ValueZero());
			X.put(E.getDirectValue(), new ValueDouble(1.0));
			return X;
		}

		return inference.queryMarginal(_TransformedNetwork.getNodes()[bnode.loc()]);
		*/
	}
}
