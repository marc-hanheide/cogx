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
 */package edu.ksu.cis.bnj.ver3.inference.approximate.ptreduction;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.util.driver.Options;

/*
 * Created on Sep 24, 2004
 *
 */

/**
 * @author Julie Thornton
 *
 */

public class KLDivergence {
	private BeliefNetwork network;
	private BeliefNetwork edgeDelNetwork;
	private int[] curInst;
	private int[] arities;
	private int[] indices;
	
	public KLDivergence(BeliefNetwork newNet) {
		network = newNet.copy();
	}
	
	private void initialize() {
		edgeDelNetwork = network.copy();
		
		curInst = new int[network.getNodes().length];
		arities = new int[network.getNodes().length];
		BeliefNode curNode;
		for (int i = 0; i < curInst.length; i++) {
			curInst[i] = 0;
			curNode = network.getNodes()[i];
			arities[i] = curNode.getDomain().getOrder();
		}
	}
	
	public double getKLDivergence(BeliefNode source, BeliefNode sink) {
		initialize();
		
		double divergence = 0.0;
		double joint, jointDelEdge;
		boolean stop = false;
		BeliefNode[] nodes = edgeDelNetwork.getNodes();
		edgeDelNetwork.disconnect(nodes[source.loc()], nodes[sink.loc()]);
		
		curInst = new int[network.getNodes().length];
		arities = new int[network.getNodes().length];
		indices = new int[network.getNodes().length];
		BeliefNode curNode;
		for (int i = 0; i < curInst.length; i++) {
			curInst[i] = 0;
			curNode = network.getNodes()[i];
			arities[i] = curNode.getDomain().getOrder();
			indices[i] = i;
		}
		
		while (!stop) {
			joint = getJointProb(curInst);
			jointDelEdge = getJointProbDelEdge(curInst);
			if (jointDelEdge != 0.0 && joint != 0.0) {
				divergence += joint*(Math.log(joint/jointDelEdge));
			}
			stop = !hasMoreInstant();
			getNextInstant();
		}
		
		return divergence;
	}
	
	private void printProb(double joint, double jointDelEdge) {
		for (int i = 0; i < curInst.length; i++) {
			System.out.println(network.getNodes()[indices[i]].getName() + ": " + curInst[i]);
		}
		System.out.println("--> " + joint + " --> " + jointDelEdge);
	}
	
	public double getOptKLDivergence(BeliefNode source, BeliefNode sink) {
		initialize();
		
		int blockSize = network.getParents(sink).length + 1;
		indices = new int[blockSize];
		curInst = new int[blockSize];
		arities = new int[blockSize];
		BeliefNode curNode;
		
		int i = 0;
		curInst[i] = 0;
		arities[i] = sink.getDomain().getOrder();
		indices[i] = sink.loc();
		for (i = 1; i < curInst.length; i++) {
			curInst[i] = 0;
			curNode = network.getParents(sink)[i-1];
			arities[i] = curNode.getDomain().getOrder();
			indices[i] = curNode.loc();
		}
		
		double divergence = 0.0;
		double probOfParents, probGivenParents, probSinkNoSource;
		boolean stop = false;
		BeliefNode[] nodes = edgeDelNetwork.getNodes();
		edgeDelNetwork.disconnect(nodes[source.loc()], nodes[sink.loc()]);
		
		while (!stop) {
			probOfParents = getProbOfParents(curInst);
			probGivenParents = getProbGivenParents(curInst);
			probSinkNoSource = getProbSinkNoSource(curInst, getParentIndex(sink, source));
			if (probGivenParents != 0.0 && probSinkNoSource != 0.0) {
				divergence += probOfParents*probGivenParents*(Math.log(probGivenParents/probSinkNoSource));
			}
			stop = !hasMoreInstant();
			getNextInstant();
		}
		
		return divergence;
	}
	
	private int getParentIndex(BeliefNode sink, BeliefNode source) {
		for (int i = 0; i < network.getParents(sink).length; i++) {
			if (network.getParents(sink)[i].getName().equals(source.getName())) {
				return i;
			}
		}
		
		return -1;
	}

	private double getProbOfParents(int[] instant) {
		double prob = 1.0;
		for (int i = 1; i < instant.length; i++) {
			int index = indices[i];
			BeliefNode node = network.getNodes()[index];
			int[] query = new int[network.getParents(node).length+1];
			query[0] = instant[i];
			int numParentInstants = 1;
			for (int j = 1; j < query.length; j++) {
				query[j] = -1;
				numParentInstants *= network.getParents(node)[j-1].getDomain().getOrder();
			}
			prob *= ((ValueDouble) node.getCPF().get(query)).getValue()/numParentInstants;
		}
		
		return prob;
	}

	private double getProbSinkNoSource(int[] instant, int sourceIndex) {
		int[] newInst = new int[instant.length-1];
		int index = 0;
		for (int i = 0; i < instant.length; i++) {
			if (i != (sourceIndex+1) || sourceIndex == -1) {
				newInst[index] = instant[i];
				index++;
			}
		}
		
		return ((ValueDouble) edgeDelNetwork.getNodes()[indices[0]].getCPF().get(newInst)).getValue();
	}

	private double getProbGivenParents(int[] instant) {
		return ((ValueDouble) network.getNodes()[indices[0]].getCPF().get(instant)).getValue();
	}

	/**
	 * gets the next instantiation of each node.
	 * I've checked this and it does generate all
	 * possible instantiations.
	 *
	 */
	private void getNextInstant() {
		for (int i = 0; i < curInst.length; i++) {
			if ((curInst[i]+1) < arities[i]) {
				curInst[i]++;
				resetInst(i-1);
				return;
			}
		}
	}
	
	private void resetInst(int stopIndex) {
		for (int i = 0; i <= stopIndex; i++) {
			curInst[i] = 0;
		}
	}
	
	private void printInst() {
		for (int i = 0; i < curInst.length; i++) {
			System.out.print(curInst[i] + " ");
		}
		System.out.println();
	}
	
	private boolean hasMoreInstant() {
		for (int i = 0; i < curInst.length; i++) {
			if ((curInst[i]+1) < arities[i]) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * gets the joint probability of the current
	 * instantiation instant.  This has also been
	 * checked for correctness.
	 * 
	 * @param instant - the current instantiation
	 * @return joint prob
	 */
	private double getJointProb(int[] instant) {
		double prob = 1.0;
		BeliefNode current;
		int[] values = null;
		for (int i = 0; i < instant.length; i++) {
			current = network.getNodes()[indices[i]];
			values = new int[network.getParents(current).length+1];
			values[0] = instant[i];
			for (int j = 0; j < network.getParents(current).length; j++) {
				values[j+1] = instant[getIndex(network.getParents(current)[j], instant)];
			}
			prob *= ((ValueDouble) current.getCPF().get(values)).getValue();
		}
		return prob;
	}
	
	private int getIndex(BeliefNode node, int[] instant) {
		for (int i = 0; i < network.getNodes().length; i++) {
			if (node.getName().equals(network.getNodes()[i].getName())) {
				return i;
			}
		}
		
		return -1;
	}
	
	/**
	 * gets the joint probability of the current instantiation
	 * given that the specified edge (source, sink) in getKLDivergence
	 * is deleted.  This has also been checked for correctness.
	 * 
	 * @param instant - the current instantiation
	 * @return the joint prob of instant with the edge deleted
	 */
	private double getJointProbDelEdge(int[] instant) {
		double prob = 1.0;
		BeliefNode current;
		int[] values = null;
		for (int i = 0; i < instant.length; i++) {
			current = edgeDelNetwork.getNodes()[i];
			values = new int[edgeDelNetwork.getParents(current).length+1];
			values[0] = instant[i];
			for (int j = 0; j < edgeDelNetwork.getParents(current).length; j++) {
				values[j+1] = instant[getIndex(edgeDelNetwork.getParents(current)[j], instant)];
			}
			prob *= ((ValueDouble) current.getCPF().get(values)).getValue();
		}
		return prob;
	}
	
	public static void main(String[] args) {
		String filename = "klExample.xml";
		BeliefNetwork bn = Options.load(filename);
		
		KLDivergence kld = new KLDivergence(bn);
		double div;
		for (int i = 0; i < bn.getNodes().length; i++) {
			for (int j = 0; j < bn.getChildren(bn.getNodes()[i]).length; j++) {
				div = kld.getOptKLDivergence(bn.getNodes()[i], bn.getChildren(bn.getNodes()[i])[j]);
				System.out.print("The divergence for the edge ");
				System.out.print(bn.getNodes()[i].getName() + "->");
				System.out.println(bn.getChildren(bn.getNodes()[i])[j].getName() + " is " + div);
			}
		}
	}
}
