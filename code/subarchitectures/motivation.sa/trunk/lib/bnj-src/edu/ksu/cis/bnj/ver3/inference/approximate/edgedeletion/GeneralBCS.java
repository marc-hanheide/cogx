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
 * Created on Dec 5, 2004
 */

package edu.ksu.cis.bnj.ver3.inference.approximate.edgedeletion;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.StringTokenizer;
import java.util.Vector;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.inference.*;
import edu.ksu.cis.bnj.ver3.inference.exact.CliqueTree;
import edu.ksu.cis.bnj.ver3.inference.exact.LS;
import edu.ksu.cis.util.RMSE;
import edu.ksu.cis.util.driver.Options;
import edu.ksu.cis.util.graph.algorithms.BCT_AlreadyTriang;
import edu.ksu.cis.util.graph.algorithms.IsConnected;
import edu.ksu.cis.util.graph.algorithms.MaximumCardinalitySearch;
import edu.ksu.cis.util.graph.algorithms.Moralization;
import edu.ksu.cis.util.graph.algorithms.PreProcTriang;
import edu.ksu.cis.util.graph.algorithms.RemoveDirectionality;
import edu.ksu.cis.util.graph.core.Graph;
import edu.ksu.cis.util.graph.core.Vertex;

/**
 * This class performs approximate inference of Bayesian networks
 * by deleting edges from the triangulated graph until the
 * maximum clique size in the triangulated graph is below a desired
 * bound.  The intial triangulation is found using maximum cardinality
 * search.
 * 
 * It chooses edges for deletion whose removal maximizes the number
 * of unnecessary moral edges.
 * 
 * For information on running this algorithm, see the comments for
 * the main method at the bottom of the file.
 * 
 * @author Julie Thornton
 */
public class GeneralBCS implements Inference {
	private BeliefNode[] beliefnodes;
	private CPF[] marginals;
	private Graph network;
	private BeliefNetwork origNetwork;
	private BeliefNetwork directedNetwork;
	public boolean klWeights = false;
	public int cliqueBound;
	private Hashtable nodeWeights;
	private Hashtable edgeWeights;
	private int[] alpha;
	private Vertex[] alphainv;
	private int biggestCliqueLoc = 0;
	private Vector moralEdges;
	private Vector[] adjacencyList;
	private int cliqueMax;
	private boolean done = false;
	private int numDeleted = 0;

	public String getName() {
		return "Approximate inference by general bounding clique sizes";
	}

	public void run(BeliefNetwork bn) {
		origNetwork = bn;
		directedNetwork = bn.copy();
		nodeWeights = new Hashtable();
		edgeWeights = new Hashtable();
		adjacencyList = new Vector[origNetwork.getNodes().length];
		
		BCT_AlreadyTriang bct = new BCT_AlreadyTriang();
		network = origNetwork.getGraph().copy();
		moralize();
		
		boundCliqueSizes();
		
		bct.setOriginal(origNetwork.getGraph());
		bct.setGraph(network);
		bct.execute(alpha, alphainv);
		
		long start = System.currentTimeMillis();
		CliqueTree CT = new CliqueTree(bct, origNetwork);
		CT.begin();
		
		marginals = CT.Marginalize();
		long stop = System.currentTimeMillis();
		System.out.println("Inference time: " + (stop-start));
		
		beliefnodes = CT.getNodes();
	}
	
	private void getNodeWeights() {
		KLDivergence kld = new KLDivergence(origNetwork);
		BeliefNode parent, child;
		double nodeWeight = 0;
		double edgeWeight;
		for (int i = 0; i < origNetwork.getNodes().length; i++) {
			parent = origNetwork.getNodes()[i];
			for (int j = 0; j < origNetwork.getChildren(parent).length; j++) {
				child = origNetwork.getChildren(parent)[j];
				if (klWeights) {
					edgeWeight = kld.getOptKLDivergence(parent, child);
				}
				else {
					edgeWeight = 1;	
				}
				edgeWeights.put(parent.getName()+"->"+child.getName(), new Double(edgeWeight));
				edgeWeights.put(child.getName()+"->"+parent.getName(), new Double(edgeWeight));
				nodeWeight += edgeWeight;
			}
			nodeWeights.put(parent.getName(), new Double(nodeWeight));
		}
	}
	
	private void moralize() {
		Moralization m = new Moralization();
		m.setGraph(network);
		m.execute();
		network = m.getGraph();
		
		RemoveDirectionality rd = new RemoveDirectionality();
		rd.setGraph(network);
		rd.execute();
		network = rd.getGraph();
		findMoralEdges();
	}
	
	private void findMoralEdges() {
		Vertex node1, node2;
		moralEdges = new Vector();
		for (int i = 0; i < network.getVertices().length; i++) {
			node1 = network.getVertices()[i];
			for (int j = 0; j < network.getParents(node1).length; j++) {
				node2 = network.getParents(node1)[j];
				if (!oldEdge(node1, node2)) {
					moralEdges.addElement(node1.getName()+"->"+node2.getName());
				}
			}
		}
	}
	
	private boolean oldEdge(Vertex node1, Vertex node2) {
		for (int i = 0; i < origNetwork.getGraph().getParents(node1).length; i++) {
			if (origNetwork.getGraph().getParents(node1)[i].getName().equals(node2.getName())) {
				return true;
			}
		}
		for (int i = 0; i < origNetwork.getGraph().getParents(node2).length; i++) {
			if (origNetwork.getGraph().getParents(node2)[i].getName().equals(node1.getName())) {
				return true;
			}
		}
		
		return false;
	}
	
	private boolean isOrigAndMoral(String name1, String name2) {
		Vertex node1 = origNetwork.getGraph().getVertices()[getVertex(name1).loc()];
		Vertex node2 = origNetwork.getGraph().getVertices()[getVertex(name2).loc()];
		Vertex current;
		Vertex[] parents;
		boolean mark1 = false;
		boolean mark2 = false;
		for (int i = 0; i < origNetwork.getGraph().getVertices().length; i++) {
			current = origNetwork.getGraph().getVertices()[i];
			parents = origNetwork.getGraph().getParents(current);
			for (int j = 0; j < parents.length; j++) {
				if (parents[j].getName().equals(node1.getName())) {
					mark1 = true;
				}
				if (parents[j].getName().equals(node2.getName())) {
					mark2 = true;
				}
			}
			if (mark1&&mark2) {
				return true;
			}
			mark1 = false;
			mark2 = false;
		}
		return false;
	}
	
	private void calcAlpha() {
		String nodeName;
		double weight;
		alpha = new int[network.getNumberOfVertices()];
		alphainv = new Vertex[alpha.length+1];
		double maxWeight = (-1)*Double.MAX_VALUE;
		String maxName = "";
		Vertex v;
		Vector usedNodes = new Vector();
		
		for (int i = 1; i < alphainv.length; i++) {
			for (Enumeration e = nodeWeights.keys(); e.hasMoreElements(); ) {
				nodeName = e.nextElement().toString();
				weight = Double.parseDouble(nodeWeights.get(nodeName).toString());
				if (weight > maxWeight && !usedNodes.contains(nodeName)) {
					maxWeight = weight;
					maxName = nodeName;
				}
			}
			v = getVertex(maxName);
			alpha[v.loc()] = i;
			alphainv[i] = v;
			
			usedNodes.addElement(maxName);
			maxWeight = (-1)*Double.MIN_VALUE;
		}
	}
	
	private Vertex getVertex(String name) {
		for (int i = 0; i < network.getVertices().length; i++) {
			if (network.getVertices()[i].getName().equals(name)) {
				return network.getVertices()[i];
			}
		}
		
		return null;
	}
	
	private void boundCliqueSizes() {
		MaximumCardinalitySearch mcs = new MaximumCardinalitySearch();
		mcs.setGraph(network);
		mcs.execute();
		alpha = mcs.getAlpha();
		alphainv = mcs.getAlphaInverse();
		while (getMaxCliqueSize() > cliqueBound && done==false) {
			shrinkBiggestClique();
		}
		System.out.println("final clique size (my computation): " + getMaxCliqueSize());
		System.out.println("edges deleted: " + numDeleted);
	}
	
	private int getMaxCliqueSize() {
		cliqueMax = 0;
		biggestCliqueLoc = -1;
		setOrigAdj();
		fillInEdges();
		
		Vertex current;
		int curMax = 1;
		for (int i = 0; i < adjacencyList.length; i++) {
			current = network.getVertices()[i];
			for (int j = 0; j < adjacencyList[i].size(); j++) {
				if (!isGreater(current, getIndex((Vertex)adjacencyList[i].elementAt(j)))) {
					curMax++;
				}
			}
			if (curMax > cliqueMax) {
				cliqueMax = curMax;
				biggestCliqueLoc = i;
			}
			curMax = 1;
		}
		return cliqueMax;
	}
	
	private int getIndex(Vertex node) {
		for (int i = 0; i < network.getVertices().length; i++) {
			if (network.getVertices()[i].getName().equals(node.getName())) {
				return alpha[i];
			}
		}
		
		return -1;
	}
	
	private void setOrigAdj() {
		Vertex current;
		for (int i = 0; i < adjacencyList.length; i++) {
			current = network.getVertices()[i];
			adjacencyList[i] = new Vector();
			for (int j = 0; j < network.getChildren(current).length; j++) {
				adjacencyList[i].addElement(network.getChildren(current)[j]);
			}
		}
	}
	
	private void fillInEdges() {
		Vertex current;
		Vector neighbors;

		for (int i = alphainv.length-1; i > 0; i--) {
			current = alphainv[i];
			neighbors = adjacencyList[current.loc()];
			fillInNode(current, i, neighbors);
		}
	}

	private void fillInNode(Vertex current, int index, Vector neighbors) {
		Vertex node1, node2;
		for (int i = 0; i < neighbors.size(); i++) {
			node1 = (Vertex)neighbors.elementAt(i);
			for (int j = i+1; j < neighbors.size(); j++) {
				node2 = (Vertex)neighbors.elementAt(j);
				if (isGreater(node1, index) && isGreater(node2, index) && notInAdj(node1.loc(), node2)) {
					adjacencyList[node1.loc()].addElement(node2);
					adjacencyList[node2.loc()].addElement(node1);
				}
			}
		}
	}
	
	private boolean notInAdj(int location, Vertex node) {
		Vector neighbors = adjacencyList[location];
		for (int i = 0; i < neighbors.size(); i++) {
			if (((Vertex)neighbors.elementAt(i)).getName().equals(node.getName())) {
				return false;
			}
		}
		
		return true;
	}

	private boolean isGreater(Vertex current, int index) {
		for (int i = 1; i < alphainv.length; i++) {
			if (alphainv[i].getName().equals(current.getName()) && i < index) {
				return true;
			}
		}

		return false;
	}
	
	private void shrinkBiggestClique() {
		Vector cliqueNodes = getCliqueNodes(biggestCliqueLoc);
		Vertex bestSource = null;
		Vertex bestSink = null;
		double bestScore = -1;
		int origNum = countAdjEdges();
		
		for (int i = 0; i < cliqueNodes.size(); i++) {
			Vertex current = (Vertex) cliqueNodes.elementAt(i);
			for (int j = 0; j < network.getChildren(current).length; j++) {
				Vertex neighbor = network.getChildren(current)[j];
				if (!moralEdges.contains(current.getName()+"->"+neighbor.getName()) &&
						!isOrigAndMoral(current.getName(), neighbor.getName())) {
					double score = countMorals(neighbor, current);
					if (score > bestScore) {
						bestScore = score;
						bestSource = current;
						bestSink = neighbor;
					}
				}
			}
		}
		if (bestSink != null) {
			numDeleted++;
			deleteExtraMorals(bestSink, bestSource);
			network.removeEdge(network.getVertices()[bestSink.loc()], 
					network.getVertices()[bestSource.loc()]);
			network.removeEdge(network.getVertices()[bestSource.loc()], 
					network.getVertices()[bestSink.loc()]);
			origNetwork.disconnect(origNetwork.getNodes()[bestSource.loc()], 
					origNetwork.getNodes()[bestSink.loc()]);
			origNetwork.disconnect(origNetwork.getNodes()[bestSink.loc()], 
					origNetwork.getNodes()[bestSource.loc()]);
			edgeWeights.remove(bestSource.getName()+"->"+bestSink.getName());
			edgeWeights.remove(bestSink.getName()+"->"+bestSource.getName());
			return;
		}
		for (int i = 0; i < network.getVertices().length; i++) {
			Vertex current = network.getVertices()[i];
			for (int j = 0; j < network.getParents(current).length; j++) {
				Vertex neighbor = network.getParents(current)[j]; 
				if (!isOrigAndMoral(current.getName(), neighbor.getName())) {
					double score = countMorals(neighbor, current);
					if (score > bestScore) {
						bestScore = score;
						bestSource = current;
						bestSink = neighbor;
					}
				}
				else {
					System.out.println(current.getName()+"-->"+neighbor.getName() + " is moral");
				}
			}
		}
		if (bestSink != null) {
			numDeleted++;
			deleteExtraMorals(bestSink, bestSource);
			network.removeEdge(network.getVertices()[bestSink.loc()], 
					network.getVertices()[bestSource.loc()]);
			network.removeEdge(network.getVertices()[bestSource.loc()], 
					network.getVertices()[bestSink.loc()]);
			origNetwork.disconnect(origNetwork.getNodes()[bestSource.loc()], 
					origNetwork.getNodes()[bestSink.loc()]);
			origNetwork.disconnect(origNetwork.getNodes()[bestSink.loc()], 
					origNetwork.getNodes()[bestSource.loc()]);
			edgeWeights.remove(bestSource.getName()+"->"+bestSink.getName());
			edgeWeights.remove(bestSink.getName()+"->"+bestSource.getName());
			return;
		}
		System.out.println("Sorry, no more edges can be deleted from the max clique");
		done = true;
	}
	
	private Vector timesMoralNeeded(String name1, String name2) {
		Vertex node1 = origNetwork.getGraph().getVertices()[getVertex(name1).loc()];
		Vertex node2 = origNetwork.getGraph().getVertices()[getVertex(name2).loc()];
		
		Vertex current;
		Vertex[] parents;
		boolean mark1 = false;
		boolean mark2 = false;
		Vector moralSinks = new Vector();
		
		for (int i = 0; i < origNetwork.getGraph().getVertices().length; i++) {
			current = origNetwork.getGraph().getVertices()[i];
			parents = origNetwork.getGraph().getParents(current);
			for (int j = 0; j < parents.length; j++) {
				if (parents[j].getName().equals(node1.getName())) {
					mark1 = true;
				}
				if (parents[j].getName().equals(node2.getName())) {
					mark2 = true;
				}
			}
			if (mark1&&mark2) {
				moralSinks.addElement(current.getName());
			}
			mark1 = false;
			mark2 = false;
		}
		return moralSinks;
	}
	
	private int countMorals(Vertex v1, Vertex v2) {
		Vertex source, sink;
		Vertex delSource = getDelSource(v1, v2);
		if (delSource.getName().equals(v1.getName())) {
			sink = v2;
		}
		else {
			sink = v1;
		}
		Vertex[] otherParents = directedNetwork.getGraph().getParents(directedNetwork.getGraph().getVertices()[sink.loc()]);
		int count = 0;
		int timesNeeded;
		String moralSink;
		Vector needed;
		
		for (int i = 0; i < otherParents.length; i++) {
			source = otherParents[i];
			needed = timesMoralNeeded(source.getName(), delSource.getName());
			timesNeeded = needed.size();
			if (moralEdges.contains(source.getName()+"->"+delSource.getName()) && 
					!oldEdge(source, delSource) &&
					timesNeeded > 0 && timesNeeded < 2) {
				moralSink = needed.elementAt(0).toString();
				if (moralSink.equals(sink.getName())) {
					count++;
				}
			}
		}
		return count;
	}
	
	private void deleteExtraMorals(Vertex v1, Vertex v2) {
		Vertex source, sink;
		Vertex delSource = getDelSource(v1, v2);
		if (delSource.getName().equals(v1.getName())) {
			sink = v2;
		}
		else {
			sink = v1;
		}
		Vertex[] otherParents = directedNetwork.getGraph().getParents(directedNetwork.getGraph().getVertices()[sink.loc()]);
		int timesNeeded;
		Vector needed;
		String moralSink;
		
		for (int i = 0; i < otherParents.length; i++) {
			source = otherParents[i];
			needed = timesMoralNeeded(source.getName(), delSource.getName());
			timesNeeded = needed.size();
			if (moralEdges.contains(source.getName()+"->"+delSource.getName()) && 
					!oldEdge(source, delSource) &&
					timesNeeded > 0 && timesNeeded < 2) {
				moralSink = needed.elementAt(0).toString();
				if (moralSink.equals(sink.getName())) {
					network.removeEdge(network.getVertices()[source.loc()], 
							network.getVertices()[delSource.loc()]);
					network.removeEdge(network.getVertices()[delSource.loc()], 
							network.getVertices()[source.loc()]);
					origNetwork.disconnect(origNetwork.getNodes()[source.loc()], 
							origNetwork.getNodes()[delSource.loc()]);
					origNetwork.disconnect(origNetwork.getNodes()[delSource.loc()], 
							origNetwork.getNodes()[source.loc()]);
					
					moralEdges.remove(source.getName()+"->"+delSource.getName());
					moralEdges.remove(delSource.getName()+"->"+source.getName());
				}
			}
		}
	}
	
	private Vertex getDelSource(Vertex v1, Vertex v2) {
		Vertex[] children1 = directedNetwork.getGraph().getChildren(directedNetwork.getGraph().getVertices()[v1.loc()]);
		
		for (int i = 0; i < children1.length; i++) {
			if (children1[i].getName().equals(v2.getName())) {
				return v1;
			}
		}
		
		return v2;
	} 
	
	private void printMorals() {
		for (int i = 0; i < moralEdges.size(); i++) {
			System.out.println(moralEdges.elementAt(i).toString());
		}
	}
	
	private Vertex findMoralNeighbor(Vertex node) {
		Vertex curNeighbor;
		for (int i = 0; i < network.getChildren(node).length; i++) {
			curNeighbor = network.getChildren(node)[i];
			return network.getVertices()[curNeighbor.loc()];
		}
		
		return null;
	}
	
	private int getNumSaved(Vertex v1, Vertex v2, int origCount) {
		Vertex current, child;
		for (int i = 0; i < adjacencyList.length; i++) {
			current = network.getVertices()[i];
			adjacencyList[i] = new Vector();
			for (int j = 0; j < network.getChildren(current).length; j++) {
				child = network.getChildren(current)[j];
				if (!(current.getName().equals(v1.getName()) && child.getName().equals(v2.getName()))
						&& !(current.getName().equals(v2.getName()) && child.getName().equals(v1.getName()))) {
					adjacencyList[i].addElement(network.getChildren(current)[j]);
				}
			}
		}
		long start = System.currentTimeMillis();
		fillInEdges();
		long stop = System.currentTimeMillis();
		int newCount = countAdjEdges();
		return (origCount - newCount);
	}
	
	private int countAdjEdges() {
		Vector neighbors;
		int count = 0;
		for (int i = 0; i < adjacencyList.length; i++) {
			neighbors = adjacencyList[i];
			count += neighbors.size();
		}
		
		return count;
	}
	
	private Vector getCliqueNodes(int biggestCliqueLoc) {
		Vector nodes = new Vector();
		nodes.addElement(network.getVertices()[biggestCliqueLoc]);
		Vector neighbors = adjacencyList[biggestCliqueLoc];
		for (int i = 0; i < neighbors.size(); i++) {
			if (isGreater((Vertex)neighbors.elementAt(i), alpha[biggestCliqueLoc])) {
				nodes.addElement(neighbors.elementAt(i));
			}
		}
		return nodes;
	}
	
	private Vertex findMaxNeighbor(Vertex node) {
		double minEdge = Double.MAX_VALUE;
		double curWeight;
		Vertex curNeighbor;
		int bestLoc = -1;
		
		for (int i = 0; i < network.getChildren(node).length; i++) {
			curNeighbor = network.getChildren(node)[i];
			if (!moralEdges.contains(node.getName()+"->"+curNeighbor.getName()) && notBridge(node, curNeighbor)) {
				curWeight = Double.parseDouble(edgeWeights.get(node.getName()+"->"+curNeighbor.getName()).toString());
				if (curWeight < minEdge) {
					minEdge = curWeight;
					bestLoc = curNeighbor.loc();
				}
				return network.getVertices()[curNeighbor.loc()];
			}
		}
		if (bestLoc == -1) {
			return null;
		}
		
		return network.getVertices()[bestLoc];
	}
	
	private boolean notBridge(Vertex node1, Vertex node2) {
		Graph tempGraph = network.copy();
		tempGraph.removeEdge(node1, node2);
		tempGraph.removeEdge(node2, node1);
		IsConnected conTest = new IsConnected();
		conTest.setGraph(tempGraph);
		conTest.execute();
		
		return conTest.connected;
	}

	public CPF queryMarginal(BeliefNode bnode) {
		for (int i = 0; i < beliefnodes.length; i++) {
			if (bnode.getName().equals(beliefnodes[i].getName())) {
				return marginals[i];
			}
		}
		
		return null;
	}

	public void printAdjList() {
		for (int i = 0; i < adjacencyList.length; i++) {
			System.out.print(network.getVertices()[i].getName() + ": ");
			for (int j = 0; j < adjacencyList[i].size(); j++) {
				System.out.print(((Vertex)adjacencyList[i].elementAt(j)).getName() + " ");
			}
			System.out.println();
		}
		System.out.println("************");
	}
	
	/*
	 * The main method runs the GeneralBCS inference algorithm
	 * on the network specified by the filename String below.  Simply
	 * change the network specified by filename to run on a different 
	 * network.
	 * 
	 * The desired bound on the maximum clique size in the triangulated
	 * graph for a network can be specified by changing the value of
	 * the cliqueBound variable, marked by the comment, "SPECIFIES CLIQUE BOUND".
	 * 
	 * For example, to set the clique bound to 5:
	 * 		tew.cliqueBound = 5;
	 * 
	 * When the algorithm runs, it will print the inference time and
	 * total time for the GeneralBCS algorithm, the
	 * inference and total time for LS, and the RMSE for the
	 * GeneralBCS algorithm.
	 */
	public static void main(String[] args) {
		String filename = "insurance.xml";
		BeliefNetwork bn = Options.load(filename);
		BeliefNetwork orig = bn.copy();

		GeneralBCS tew = new GeneralBCS();
		
		//SPECIFIES CLIQUE BOUND
		tew.cliqueBound = 5;
		
		long start = System.currentTimeMillis();
		tew.run(bn);
		long stop = System.currentTimeMillis();
		System.out.println("total time: " + (stop-start));
		
		LS ls = new LS();
		start = System.currentTimeMillis();
		ls.run(orig);
		stop = System.currentTimeMillis();
		System.out.println("LS total time: " + (stop-start));
		
		BeliefNode[] nodes = bn.getNodes();
		CPF[] lsMarginals = new CPF[nodes.length];
		CPF[] approxMarginals = new CPF[nodes.length];
		for (int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			lsMarginals[i] = ls.queryMarginal(x);
			approxMarginals[i] = tew.queryMarginal(x);
		}
		System.out.println("The RMSE is: " + RMSE.computeRMSE(lsMarginals, approxMarginals).getExpr());
	}
}
