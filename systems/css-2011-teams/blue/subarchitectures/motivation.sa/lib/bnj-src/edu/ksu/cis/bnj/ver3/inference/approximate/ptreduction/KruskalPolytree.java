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
 * Created on Aug 22, 2004
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.ptreduction;

import java.util.StringTokenizer;
import java.util.Vector;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.bnj.ver3.inference.exact.LS;
import edu.ksu.cis.bnj.ver3.inference.exact.Pearl;
import edu.ksu.cis.util.driver.Options;

/**
 * @author Julie Thornton
 *
 * This class uses Kruskal's algorithm to find the best polytree
 * for a Bayesian network.  It then uses Pearl's algorithm on that
 * polytree to get the marginal probabilities.  It can optionally assign
 * edge weights according to the amount of information lost in the CPTs
 * by deleting an edge, or it can use uniform edge weights.
 */
public class KruskalPolytree implements Inference {
	private CPF[] marginals = null;
	private BeliefNode[] beliefnodes;
	private KLDivergence kl = null;
	
	public final int NO_WEIGHTS = 0;
	public final int SKEW_WEIGHTS = 1;
	public final int KL_WEIGHTS = 2;
	public int weightEdges = NO_WEIGHTS;  //what kind of weighting to be done
	
	public String getName() {
		return "Find best polytree with Kruskal's Algorithm";
	}

	public void run(BeliefNetwork bn) {
		beliefnodes = bn.getNodes();
		if (weightEdges == KL_WEIGHTS) {
			kl = new KLDivergence(bn);
		}

		//forest is a vector where each element is a list of 
		//node names in that tree
		Vector forest = new Vector();
		
		//initially, each tree is a single node
		Vector tree = null;
		for (int i = 0; i < beliefnodes.length; i++) {
			tree = new Vector();
			tree.addElement(beliefnodes[i].getName());
			forest.addElement(tree);
		}
		
		//allEdges holds a list of all edges in the network
		//where each edge is listed in sourceName,sinkName fashion
		//The vector is ordered by edge weight (skewness 
		//of probabilities affected by deleting that edge)
		Vector allEdges = getAllEdges(bn.copy());
		
		//main loop of Kruskal's algorithm
		String currentEdge = null;
		while (!allEdges.isEmpty()) {
			currentEdge = (String) allEdges.remove(0);
			if (!addEdgeToForest(currentEdge, forest, bn)) {
				deleteEdge(currentEdge, bn);
				//System.out.println(currentEdge);
			}
		}
		long time1 = System.currentTimeMillis();
		Pearl pearl = new Pearl();
		pearl.run(bn);
		System.out.println("Pearl inference time: " + (System.currentTimeMillis() - time1));
		beliefnodes = bn.getNodes();

		marginals = new CPF[beliefnodes.length];
		for (int i = 0; i < beliefnodes.length; i++) {
			marginals[i] = pearl.queryMarginal(beliefnodes[i]);
		}
	}
	
	private Vector getAllEdges(BeliefNetwork netCopy) {
		Vector allEdges = new Vector();
		Vector scores = new Vector();
		BeliefNode[] children = null;
		BeliefNode[] nodes = netCopy.getNodes();
		BeliefNetwork freshCopy = null;
		
		for (int i = 0; i < nodes.length; i++) {
			children = netCopy.getChildren(nodes[i]);
			for (int j = 0; j < children.length; j++) {
				allEdges.addElement(nodes[i].getName() + "," + children[j].getName());
				if (weightEdges == SKEW_WEIGHTS) {
					CPF oldCPF = children[j].getCPF();
					BeliefNode[] domain = oldCPF.getDomainProduct();
					freshCopy = netCopy.copy();
					freshCopy.disconnect(freshCopy.getNodes()[nodes[i].loc()], 
									     freshCopy.getNodes()[children[j].loc()]);
					CPF newCPF = freshCopy.getNodes()[children[j].loc()].getCPF();
					scores.addElement(getScore(domain, oldCPF, newCPF, nodes[i].getName())+"");
				}
				else if (weightEdges == KL_WEIGHTS) {
					double weight = kl.getOptKLDivergence(nodes[i], children[j]);
					scores.addElement(weight+"");
				}
			}
		}
		
		if (weightEdges != NO_WEIGHTS) {
			sortEdges(allEdges, scores);
		}
		return allEdges;
	}
	
	private void printEdgeWeights(Vector edges, Vector scores) {
		for (int i = 0; i < edges.size(); i++) {
			System.out.print(edges.elementAt(i).toString() + ": ");
			System.out.println(scores.elementAt(i).toString());
		}
		
		System.out.println();	
	}
	
	private double getScore(BeliefNode[] domain, CPF oldCPF, CPF newCPF, String delParent) {
		//find index of delParent in domain
		//find number of values delParent takes
		int delIndex = 0;
		int valCount = 0;
		for (int i = 0; i < domain.length; i++) {
			if (delParent.equals(domain[i].getName())) {
				delIndex = i;
				valCount = domain[i].getDomain().getOrder();
				break;
			}
		}
		
		//loop through all CPF entries, update score
		double tempScore = 0;
		for(int i = 0; i < oldCPF.size(); i++)
		{
			int[] q = oldCPF.realaddr2addr(i);
			int[] newQ = newCPF.getSubQuery(q, domain);
			
			double oldEntry = Double.parseDouble(oldCPF.get(i).getExpr());
			double newEntry = Double.parseDouble(newCPF.get(newQ).getExpr());

			tempScore += Math.abs(oldEntry - newEntry);
		}
		
		tempScore /= valCount;
		
		return tempScore;
	}
	
	private void sortEdges(Vector edges, Vector scores) {
		double max = -1.0;
		String curEdge = null;
		int bestIndex = -1;
		String tempEdge = null;
		double curScore = 0.0;
		double tempScore = 0.0;
		for (int i = 0; i < scores.size(); i++) {
			for (int j = i; j < scores.size(); j++) {
				curScore = (new Double(scores.elementAt(j).toString())).doubleValue();
				if (curScore > max) {
					max = curScore;
					bestIndex = j;
				}
			}
			tempEdge = edges.elementAt(bestIndex).toString();
			edges.setElementAt(edges.elementAt(i), bestIndex);
			edges.setElementAt(tempEdge, i);
			
			tempScore = (new Double(scores.elementAt(bestIndex).toString())).doubleValue();
			scores.setElementAt(scores.elementAt(i), bestIndex);
			scores.setElementAt(tempScore+"", i);
			
			max = -1.0;
			curEdge = null;
			bestIndex = -1;
			tempEdge = null;
			curScore = 0.0;
			tempScore = 0.0;
		}
	}

	private void deleteEdge(String currentEdge, BeliefNetwork bn) {
		String source = getSource(currentEdge);
		String sink = getSink(currentEdge);
		BeliefNode sourceNode = null;
		BeliefNode sinkNode = null;
		BeliefNode[] nodes = bn.getNodes();
		
		for (int i = 0; i < bn.getNodes().length; i++) {
			if (bn.getNodes()[i].getName().equals(source)) {
				sourceNode = bn.getNodes()[i];
				for (int j = 0; j < bn.getChildren(sourceNode).length; j++) {
					if (bn.getChildren(sourceNode)[j].getName().equals(sink)) {
						sinkNode = bn.getChildren(sourceNode)[j];
					}
				}
			}
		}
		
		bn.disconnect(nodes[sourceNode.loc()], nodes[sinkNode.loc()]);
	}

	private boolean addEdgeToForest(String currentEdge, Vector forest, BeliefNetwork network) {
		String source = getSource(currentEdge);
		String sink = getSink(currentEdge);
		Vector sourceTree = null;
		Vector sinkTree = null;
		
		//loop through every possible combination of two trees
		//where first tree has source and second tree has sink
		for (int i = 0; i < forest.size(); i++) {
			sourceTree = (Vector) forest.elementAt(i);
			if (inTree(source, sourceTree)) {
				for (int j = 0; j < forest.size(); j++) {
					sinkTree = (Vector) forest.elementAt(j);
					if (i != j && inTree(sink, sinkTree)) {
						if (combineTrees(sourceTree, source,
										 sinkTree, sink,
										 forest, network)) {
							return true;
						}
					}
				}
			}
		}
		
		return false;
	}
	
	//returns true if nodeName is an element in tree
	private boolean inTree(String nodeName, Vector tree) {
		for (int i = 0; i < tree.size(); i++) {
			if (tree.elementAt(i).toString().equals(nodeName)) {
				return true;
			}
		}
		
		return false;
	}

	//returns false if there is no edge between source and sink
	//returns true if there is, and combines the two trees in the
	//forest
	private boolean combineTrees(Vector tree1, String source, Vector tree2, String sink,
								 Vector forest, BeliefNetwork network) {
		
		BeliefNode[] nodes = network.getNodes();
		BeliefNode[] children = null;
		for (int i = 0; i < nodes.length; i++) {
			if (nodes[i].getName().equals(source)) {
				children = network.getChildren(nodes[i]);
				for (int j = 0; j < children.length; j++) {
					if (children[j].getName().equals(sink)) {
						for (int k = 0; k < tree2.size(); k++) {
							tree1.addElement(tree2.elementAt(k));
						}
						forest.remove(tree2);
						return true;
					}
				}
			}
		}
	
		return false;
	}

	private String getSink(String currentEdge) {
		StringTokenizer st = new StringTokenizer(currentEdge, ",");
		st.nextToken();
		return st.nextToken();
	}

	private String getSource(String currentEdge) {
		StringTokenizer st = new StringTokenizer(currentEdge, ",");
		return st.nextToken();
	}
	
	public CPF queryMarginal(BeliefNode bnode) {
		for (int i = 0; i < beliefnodes.length; i++) {
			if (bnode.getName().equals(beliefnodes[i].getName())) {
				return marginals[i];
			}
		}
		
		return null;
	}
	
	public double computeRMSE(CPF[] lsMarginals, CPF[] approxMarginals) {
		double rmse = 0;
		int count = 0;
		
		for (int i = 0; i < marginals.length; i++) {
			CPF ls = lsMarginals[i];
			CPF pt = approxMarginals[i];
			
			StringTokenizer stLS = new StringTokenizer(Options.getString(ls));
			StringTokenizer stPT = new StringTokenizer(Options.getString(pt));
			
			//the strings are of the form:
			//{WetGrass=false} := 0.3529000000000001
			//{WetGrass=true} := 0.6471
			while (stLS.hasMoreTokens()) {
				//get past the instantiation {Name=value} and :=
				stLS.nextToken();
				stLS.nextToken();
				
				stPT.nextToken();
				stPT.nextToken();
				
				double valLS = Double.parseDouble(stLS.nextToken());
				double valPT = Double.parseDouble(stPT.nextToken());
				rmse += Math.pow(Math.abs(valLS - valPT), 2);
				count++;
			}
		}
		
		rmse /= count;
		rmse = Math.sqrt(rmse);
		return rmse;
	}
	
	public static void main(String[] args) {
		String filename = "cpcs-54.xml";
		BeliefNetwork bn = Options.load(filename);
		BeliefNetwork orig = bn.copy();
		
		if(bn == null) {
			System.out.println("file not found");
			return;
		}
		
		long start = 0;
		long stop = 0;
		
		KruskalPolytree kp = new KruskalPolytree();
		kp.weightEdges = kp.KL_WEIGHTS;
		
		start = System.currentTimeMillis();
		kp.run(bn);
		stop = System.currentTimeMillis();	
		System.out.println("Kruskal Polytree time: " + (stop-start));
		
		BeliefNode[] nodes = bn.getNodes();
		
		/*for(int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			System.out.println(x.getName() + ":\n" + Options.getString(kp.queryMarginal(x)) + "\n");
		}*/
		
		
		LS ls = new LS();
		//bn = Options.load(filename);
		
		start = System.currentTimeMillis();
		ls.run(orig);
		stop = System.currentTimeMillis();
		System.out.println("LS time: " + (stop-start));
		
		nodes = bn.getNodes();
		CPF[] lsMarginals = new CPF[nodes.length];
		CPF[] approxMarginals = new CPF[nodes.length];
		for (int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			lsMarginals[i] = ls.queryMarginal(x);
			approxMarginals[i] = kp.queryMarginal(x);
		}
		System.out.println("The RMSE is: " + kp.computeRMSE(lsMarginals, approxMarginals));
	}

}
