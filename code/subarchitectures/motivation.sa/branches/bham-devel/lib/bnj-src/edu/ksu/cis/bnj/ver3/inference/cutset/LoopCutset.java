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
 * Created on 2004/07/28
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package edu.ksu.cis.bnj.ver3.inference.cutset;
import java.util.ArrayList;

import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
//import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.util.graph.visualization.VisualizationController;

/**
 * @author Chris Meyer
 *
 * LoopCutset needs to be renamed to KnotCutset.
 * This class uses Knot Confitioning to make inference on a network faster without a decrease in accuaracy.
 */
public class LoopCutset implements Inference {

/**
 * 
 * @param bn - The Belief Network we're working with
 * @param VC - We'll need this for the Visualization tool later on.
 */
	public void run(BeliefNetwork bn, VisualizationController VC) {
		
	}
	
	/**
	 * Well, at least this method returns the right term for this class
	 */
	public String getName() {
		return "Knot Cutset";
	}

/**
 * @param bn - The Belief Network we're working with
 */
	public void run(BeliefNetwork bn){
		
	}
	
	/**
	 *  Still need to check for null parents / children so no disconnects occur
	 *  Very bad for single-parent to many-children networks
	 *  NOTE: This method is primarily for visualization as of 2004/07/29.  This method will display what visually 
	 *        is happening to the network.  The CPF and inference will be integrated shortly.
	 * @param BND - The Belief Network Copy we're working with.  This isn't the real network yet, need to develope this 
	 * class more.
	 * @return - Returns an altered BeliefNetwork based on the KnotCutset's results
	 * TODO: Make this method work from the inside->out, rather than the outside->in (Current State)
	 */

	public static BeliefNetwork KC (BeliefNetwork BND) {
		BeliefNode[] nodes = BND.getNodes();
		int parentCount = 1;
		boolean KnotProc = false;
		if (nodes!=null) KnotProc = true;
		
		ArrayList path = new ArrayList();
		
		while (KnotProc) {
			
			//First, we choose a node.  Why not the first one?
			for (int i = 0; i < nodes.length; ++i) {
			try {
				//We check to see if a common event occurs, having more than 1 child node.  This is usually an indicator
				//of potential substitution.  Splitting a node with 2 or more parents gets tricky, so we'll check that
				//later.
				if (((BND.getParents(nodes[i])).length < 2) && (BND.getChildren(nodes[i]).length > 1) ) {
					BeliefNode fosterParent = new BeliefNode("Foster Node " + parentCount++, nodes[i].getDomain());
					BeliefNode[] adoptedChildren = BND.getChildren(nodes[i]);
					
					//The case of a node having no parent (Easy split)
					if (BND.getParents(nodes[i]).length <= 0) {
						BND.addBeliefNode(fosterParent);
						BND.connect(fosterParent, adoptedChildren[0]);
						BND.disconnect(nodes[i], adoptedChildren[0]);
						--i;
					}
					
					//Case where a parent is present, just a little bit trickier.
					else {
						BeliefNode[] oldParents = BND.getParents(nodes[i]);
						BND.connect(oldParents[0], fosterParent);
						BND.disconnect(oldParents[0], nodes[i]);
						BND.connect(fosterParent, adoptedChildren[0]);
						BND.disconnect(nodes[i], adoptedChildren[0]);
						--i;
					}
				}
				nodes = BND.getNodes();
			}
			//Exceptions occur when we've split and added nodes, and null-children ghosts or seperation occurs
			/*
			 * TODO: Check for network seperation
			 */
		    catch (NullPointerException e) {
		    	//System.out.println("Knot Cutset would have caused a failure on this network.  To Be fixed soon.");
		    }
			}
			KnotProc = false;
		}
		
		return BND;
	}
/**
 *  Useless method for now, but necessary to implement Inference which will eventually be tied in
 */
	public CPF queryMarginal(BeliefNode bnode) {
		return new CPF();
	}

}
