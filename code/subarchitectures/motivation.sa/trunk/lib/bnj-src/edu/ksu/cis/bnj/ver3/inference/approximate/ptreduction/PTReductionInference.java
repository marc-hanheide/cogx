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
 *
 * Created on Jul 12, 2004
 *
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.ptreduction;

import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.inference.*;
import edu.ksu.cis.bnj.ver3.inference.approximate.sampling.AIS;
import edu.ksu.cis.bnj.ver3.inference.exact.*;
import edu.ksu.cis.util.driver.Options;
import java.util.*;


/**
 * @author Julie Thornton
 *
 */
public class PTReductionInference implements Inference {
	
	private CPF[] marginals = null;
	private BeliefNetwork finalGraph;
	private BeliefNode[] beliefnodes;
	private int beamWidth;
	
	public String getName() {
		return "Inference by Reduction to Polytree";
	}
	
	public void run(BeliefNetwork bn) {
		BeliefNetwork bestPoly = null;
		double bestScore = Double.MAX_VALUE;
		Vector openList = new Vector();
		Vector closedList = new Vector();
		PartialPolytree orig = new PartialPolytree(bn);
		openList.addElement(orig);
		
		while (!openList.isEmpty()) {
			PartialPolytree curGraph = (PartialPolytree) openList.elementAt(0);
			BeliefNetwork g = curGraph.getNetwork();
			System.out.println(g.getGraph().getNumberOfEdges());
			if (g.getGraph().getNumberOfEdges() == (g.getGraph().getNumberOfVertices() - 1)) {
				double score = curGraph.getScore();
				if (score < bestScore) {
					System.out.println("score: " + score);
					System.out.println("Deleted edges: " + curGraph.deleted);
					bestPoly = g.copy();
					bestScore = score;
				}
			}
			else {
				BeliefNode[] nodes = g.getNodes();
				for (int i = 0; i < nodes.length; i++) {
					BeliefNode[] children = g.getChildren(nodes[i]);
					for (int j = 0; j < children.length; j++) {
						PartialPolytree pt = (PartialPolytree) curGraph.clone();
						BeliefNode[] curNodes = pt.getNetwork().getNodes();
						BeliefNode[] curChildren = pt.getNetwork().getChildren(curNodes[i]);
						if (pt.deleteEdge(curNodes[i], curChildren[j])) {
							if (!inList(closedList, pt)) {
								openList.addElement(pt);
							}
						}
					}
				}
			}
			
			openList.removeElementAt(0);
			closedList.addElement(curGraph);
			sortList(openList);
			pruneList(openList);
		}
		long time1 = System.currentTimeMillis();
		Pearl pearl = new Pearl();
		pearl.run(bestPoly);
		System.out.println("Pearl inference time: " + (System.currentTimeMillis() - time1));
		
		beliefnodes = new BeliefNode[bestPoly.getNodes().length];

		beliefnodes = bestPoly.getNodes();
		marginals = new CPF[beliefnodes.length];
		for (int i = 0; i < beliefnodes.length; i++) {
			marginals[i] = pearl.queryMarginal(beliefnodes[i]);
		}
	}
	
	public void setBeamWidth(int width) {
		beamWidth = width;
	}
	
	private void sortList(Vector openList) {
		double min = Double.MAX_VALUE;
		PartialPolytree curPoly = null;
		int bestIndex = -1;
		PartialPolytree tempPoly = null;
		for (int i = 0; i < openList.size(); i++) {
			for (int j = i; j < openList.size(); j++) {
				curPoly = (PartialPolytree) openList.elementAt(j);
				if (curPoly.getScore() < min) {
					min = curPoly.getScore();
					bestIndex = j;
				}
			}
			tempPoly = (PartialPolytree) openList.elementAt(bestIndex);
			openList.setElementAt(openList.elementAt(i), bestIndex);
			openList.setElementAt(tempPoly, i);
			
			min = Double.MAX_VALUE;
			curPoly = null;
			bestIndex = -1;
			tempPoly = null;
		}
	}
	
	private void pruneList(Vector openList) {
		int size = openList.size();
		int index = 0;
		while (index < size) {
			if (index >= beamWidth) {
				openList.removeElementAt(index);
				size--;
			}
			else {
				index++;
			}
		}
	} 
	
	private void printListInfo(Vector openList) {
		System.out.println("Printing open list scores:");
		for (int i = 0; i < openList.size(); i++) {
			System.out.println(((PartialPolytree)openList.elementAt(i)).getScore());
		}
		System.out.println("**************\n");
	}
	
	private boolean inList(Vector closedList, PartialPolytree pt) {
		Vector ptDeleted = pt.deletedVector;
		
		for (int i = 0; i < closedList.size(); i++) {
			PartialPolytree curPoly = (PartialPolytree) closedList.elementAt(i);
			Vector closedDeleted = (Vector) curPoly.deletedVector.clone();
			
			if (ptDeleted.size() != closedDeleted.size()) {
				for (int j = 0; j < ptDeleted.size(); j++) {
					String edge = ptDeleted.elementAt(j).toString();
					closedDeleted.removeElement(edge);
				}
				
				if (closedDeleted.size() == 0) {
					return true;
				}
			}
		}
	
		return false;
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
	
	public CPF queryMarginal(BeliefNode bnode) {
		for (int i = 0; i < beliefnodes.length; i++) {
			if (bnode.getName().equals(beliefnodes[i].getName())) {
				return marginals[i];
			}
		}
		
		return null;
	}
	
	public void printNetworkStats(BeliefNetwork bn) {
		System.out.println("Report for " + bn.getName());
		System.out.println("Nodes: " + bn.getGraph().getNumberOfVertices());
		System.out.println("Edges: " + bn.getGraph().getNumberOfEdges());
		
		double sum = 0.0;
		int max = 0;
		for (int i = 0; i < bn.getNodes().length; i++) {
			sum = sum + bn.getNodes()[i].getCPF().size();
			if (max < bn.getNodes()[i].getCPF().size()) {
				max = bn.getNodes()[i].getCPF().size();
			}
		}
		
		System.out.println("Average CPT size: " + (sum/bn.getNodes().length));
		System.out.println("Biggest CPT size: " + max);
	} 

	public static void main(String[] args) {
		String filename = "cpcs-54.xml";
		BeliefNetwork bn = Options.load(filename);
		
		if(bn == null) {
			System.out.println("file not found");
			return;
		}
		
		long start = 0;
		long stop = 0;
		
		PTReductionInference ptInf = new PTReductionInference();
		ptInf.setBeamWidth(1);
		
		start = System.currentTimeMillis();
		ptInf.run(bn);
		stop = System.currentTimeMillis();	
		System.out.println("PTReductionInference time: " + (stop-start));
		
		BeliefNode[] nodes = bn.getNodes();
		
		/*for(int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			System.out.println(x.getName() + ":\n" + Options.getString(ptInf.queryMarginal(x)) + "\n");
		}*/
		
		LS ls = new LS();
		
		start = System.currentTimeMillis();
		ls.run(bn);
		stop = System.currentTimeMillis();
		System.out.println("LS time: " + (stop-start));
		
		nodes = bn.getNodes();
		CPF[] lsMarginals = new CPF[nodes.length];
		CPF[] approxMarginals = new CPF[nodes.length];
		for (int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			lsMarginals[i] = ls.queryMarginal(x);
			approxMarginals[i] = ptInf.queryMarginal(x);
		}
		System.out.println("The RMSE is: " + ptInf.computeRMSE(lsMarginals, approxMarginals));
		
		AIS ais = new AIS();
		
		start = System.currentTimeMillis();
		ais.run(bn);
		stop = System.currentTimeMillis();
		System.out.println("AIS time: " + (stop-start));
		
		//still need AIS RMSE
		nodes = bn.getNodes();
		approxMarginals = new CPF[nodes.length];
		for (int i = 0; i < nodes.length; i++) {
			BeliefNode x = nodes[i];
			approxMarginals[i] = ais.queryMarginal(x);
		}
		System.out.println("The AIS RMSE is: " + ptInf.computeRMSE(lsMarginals, approxMarginals));
	}
}
