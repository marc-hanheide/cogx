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
 *//**
 * 
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.sampling;

import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.util.graph.core.*;
/**
 * @author Andrew King
 *
 */
public class AISBN implements Inference {
	
	private BeliefNetwork network;
	private Graph graph;
	private Vertex[] graph_vertices;
	private BeliefNode[] nodes;
	private double[][] ICPT;

	/* (non-Javadoc)
	 * @see edu.ksu.cis.bnj.ver3.inference.Inference#getName()
	 */
	public String getName() {
		// TODO Auto-generated method stub
		return "AIS-BN";
	}

	/* (non-Javadoc)
	 * @see edu.ksu.cis.bnj.ver3.inference.Inference#run(edu.ksu.cis.bnj.ver3.core.BeliefNetwork)
	 */
	public void run(BeliefNetwork bn) {
		network = bn;
		graph = bn.getGraph();
		nodes = bn.getNodes();
		
	}

	/* (non-Javadoc)
	 * @see edu.ksu.cis.bnj.ver3.inference.Inference#queryMarginal(edu.ksu.cis.bnj.ver3.core.BeliefNode)
	 */
	public CPF queryMarginal(BeliefNode bnode) {
		// TODO Auto-generated method stub
		return null;
	}
	/* (non-Javadoc)
	 * builds an ICPT table for all of the nodes
	 */
	private void initializeICPT(){
		ICPT = new double[nodes.length][];
		for(int i = 0; i < nodes.length; i++){
			ICPT[i] = new double[nodes[i].getCPF().getDomainProduct().length];
			for(int c = 0; c < ICPT[i].length; c++){
				ICPT[i][c] = Double.parseDouble(nodes[i].getCPF().get(c).getExpr());
			}
		}
	}
	/* (non-Javadoc)
	 *  This method is our implementation of the frist Heuristic initializtion function 
	 */
	private void initialize1(){
		for(int i = 0; i < nodes.length; i++){
			if(nodes[i].hasEvidence()){
				//get the parent nodes
				//for each parent node, get the number of outcomes of those nodes
				//and normalize prob distribution
				BeliefNode[] parents = network.getParents(nodes[i]);
				int outcomes = nodes[i].getDomain().getOrder(); //get the number of outcomes for the evidence node
				int cpf_size = nodes[i].getCPF().size();
				int interval = outcomes / cpf_size; 
				int ev_index = ((DiscreteEvidence)(nodes[i].getEvidence())).getDirectValue();
				double threshold = 1 / (2 * outcomes);
				double ev_prob = 0.0;
				
				for(int c = 0; c < parents.length; c++){
					
				}
			}
		}	
	}
	

}
