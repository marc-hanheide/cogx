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
 *//*
 * Created on Jul 27, 2004
 */
package edu.ksu.cis.bnj.ver3.inference.exact;

import java.util.Vector;

import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.util.driver.Options;

/*!
 * \author Julie Thornton
 * \Pearl's exact inference algorithm for polytrees
 */
public class Pearl implements Inference {
	private BeliefNetwork network;
	private PropNode[] nodes;
	
	/*! Get the name of this Inference Algorithm
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::getName()
	 */
	public String getName() {
		return "Pearl's Algorithm for Polytrees";
	}
	
	/*! Run the Inference
	 * \param[in] bn the Belief Network to run inference on
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::run(edu.ksu.cis.bnj.ver3.core.BeliefNetwork)
	 */
	public void run(BeliefNetwork bn) {
		network = bn;
		setup();
		
		//have the roots send their pi messages, and 
		//propagate them down
		Vector openList = getRoots();
		PropNode curNode = null;
		while (!openList.isEmpty()) {
		    curNode = (PropNode) openList.remove(0);
		    curNode.sendPiMessages(-1);
		}
		
		//start propagation from evidence nodes
		openList = getEvidence();
		while (!openList.isEmpty()) {
		    curNode = (PropNode) openList.remove(0);
		    curNode.propagateEvidence();
		}
		
		//printResults();
	}
	
	private Vector getEvidence() {
	    Vector evidence = new Vector();
	    for (int i = 0; i < nodes.length; i++) {
	        if (nodes[i].getNode().hasEvidence()) {
	            evidence.addElement(nodes[i]);
	        }
	    }
	    
	    return evidence;
	}
	
	private Vector getRoots() {
	    Vector roots = new Vector();
	    for (int i = 0; i < nodes.length; i++) {
	        if (network.getParents(network.getNodes()[i]).length == 0) {
	            roots.addElement(nodes[i]);
	        }
	    }
	    
	    return roots;
	}
	
	private void setup() {
	    BeliefNode[] bnNodes = network.getNodes();
	    nodes = new PropNode[bnNodes.length];
	    
	    for (int i = 0; i < bnNodes.length; i++) {
	        PropNode prop = new PropNode(bnNodes[i], network);
	        nodes[i] = prop;
	    }
	    
	    for (int i = 0; i < nodes.length; i++) {
	        nodes[i].initialize(nodes);
	    }
	}
	
	/*! Query for a Marginal
	 * \param[in] bnode the Belief Node to query the marginal of
	 * \see edu.ksu.cis.bnj.ver3.inference.Inference::queryMarginal()
	 */
	public CPF queryMarginal(BeliefNode node) {
		for (int i = 0; i < nodes.length; i++) {
		    if (nodes[i].getNode().getName().equals(node.getName())) {
		        return nodes[i].getMarginals();
		    }
		}
		return null;
	}
	
	/*! Outputs the marginals for the belief network
	 */
	public void printResults() {
		BeliefNode[] nodes = network.getNodes();
		for(int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			System.out.println(x.getName() + ":\n" + 
							   Options.getString(queryMarginal(nodes[i])) + "\n");
		}
	}
}
