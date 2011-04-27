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
 * Created on Jul 16, 2004
 *
 *
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.sampling;

import java.util.*;
import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.values.*;
import edu.ksu.cis.bnj.ver3.inference.*;
import edu.ksu.cis.util.graph.visualization.*;
import edu.ksu.cis.util.graph.visualization.operators.*;




/**
 * @author Andrew King
 *
 * This class contains the AIS code. The bulk of this algorithms implementation
 * was lifted from the BNJ v1 source.  To operate, this class takes a BNJv3 network 
 * and translates it into v1 format.  There is a runtime penalty associated with this;
 * the penalty is about linear in the number of nodes in the network.  Also, after 
 * the algorithm is done executing, it must package the marginals back up into 
 * V3 style CPF objects. This runtime penalty is also about linear about the number of
 * nodes in the network.
 * 	For those individuals who want to do testing of AIS I have put comments in the
 * code the denote the start and end of the code that is actually doing AIS work. These
 * would be the places where you would sample ticks if you wanted to record the running
 * time of AIS.
 * 
 * This uses alot (alot) of code developed by Julie Thornton and others for BNJv1
 */
public class AIS implements Inference{

	
	private BeliefNetwork v3network;
	public Vector[] ICPT;
	private int m = 2000;
	private int l = 200;
	private Vector nodechanges;
	private double learningrate;
	private double weight;
	public boolean old;
	public Vector probabilityArray;
	public boolean changeProbs = true;
	public boolean evidence_specified = false;
	private BeliefNode[] nodes;
	public BBNLifted network;
	private NodeLifted[] oldnodes;
	public EvaluatorLifted eval;
	private Vector exactProbs;
	private BeliefNode[] nodes_formarginals;
	
	Random generator;
	
	VisualizationController VC;

	public AIS(){}
	
	public AIS(VisualizationController _VC){
		VC = _VC;
	}
	
	/**
		 * what is the name of this inference algorithm
		 * 
		 * @return the name
		 */
		 
		public String getName(){
			return("AIS");
		}
		/**
		 * run this algorithm on the belief network
		 * 
		 * @param bn
		 *            the bayesian network
		 */
	 
		public void run(BeliefNetwork bn){
			//First unroll all the CPFs and put them in the array of prob vectors(ICPT)
			v3network = bn;
			generator = new Random();
			nodes = v3network.getNodes();
			evidence_specified = checkNodesForEvidence(nodes);
			network = v3tov1BBN(bn);
		   
			int net_size = v3network.getGraph().getNumberOfVertices();
			ICPT = new Vector[net_size];
			nodes_formarginals = new BeliefNode[net_size];
			for (int i = 0; i < network.size(); i++)
					   {
						   Vector v     = new Vector();
						   NodeLifted n       = network.getNodeAt(i);
						   Vector probs = n.getProbabilities();
						   NodeLifted current = oldnodes[i];

						   for (int j = 0; j < probs.size(); j++)
						   {
							   v.addElement(probs.elementAt(j));
						   }

						   ICPT[i]  = v;
					   }
			
			eval = new EvaluatorLifted(m, network, v3network, "ais", this);
			generateAllSamples();
			packageMarginals();
			//Quick dirty print of computed probs -change for release :)
			//for(int i = 0; i < probabilityArray.size(); i++){
			//	if(probabilityArray.get(i) != null){
			//		Vector node_probs = (Vector)probabilityArray.get(i);
			//		System.out.println("Node: " + i + " : " + node_probs.size());
			//		for(int c = 0; c < node_probs.size(); c++){
			//			System.out.println("\t\t Probs " + c + " " + node_probs.get(c) );
			//		}
			//	}
			//}
			
		}
		/**
		 * after running, query a marginal for a node
		 * 
		 * @param bnode -
		 *            a belief node we want to query
		 * @return the cpf that contains the marginal
		 */
		public CPF queryMarginal(BeliefNode bnode){
			CPF marginal_cpf = new CPF();
			for(int i = 0; i < nodes_formarginals.length; i++){
				//System.out.println("node: " + i + " " + nodes_formarginals[i] );
				if(bnode.getName().equals(nodes_formarginals[i].getName())){
					marginal_cpf = nodes_formarginals[i].getCPF();
					
				}
				else{}
			}
			return marginal_cpf;
		}
		/**
		 * this takes a cpf and "unrolls" it, into a string of values in the order bnjv1 needs it
		 * @param cpf
		 * @return
		 */
		public Vector unrollCPF(CPF cpf){
			int size = cpf.size();
			Vector probs = new Vector(size);
			for(int i = 0; i < size; i++){
				ValueDouble vald = (ValueDouble)cpf.get(i);
				Double d = new Double(vald.getValue());
				probs.addElement(d);
			}
			return probs;
		}
		
		public void rollupCPF(Vector vect, CPF cpf){
			int size = vect.size();
			for(int i = 0; i < size; i++){
				Double d = (Double)vect.get(i);
				ValueDouble vald = new ValueDouble(d.doubleValue());
				cpf.put(i, vald);
			}
			
		}
		
	
		
		
/** The magnitude of the following hack is comparable to that of the National Debt :) 
 *  v3tov1BBN takes a BNJv3 BeliefNetwork and converts it into 
 *  a BNJv1 style network. This is where the bulk of the initialization
 * (NON-AIS) runtime is spent.
 * */		
		
		private BBNLifted v3tov1BBN(BeliefNetwork new_net){
			Hashtable nodestore = new Hashtable(); 
			BBNLifted oldstyle_net = new BBNLifted();
			BeliefNode[] v3nodes = new_net.getNodes();
			Vector oldstyle_nodelist = new Vector();
			int net_size = v3nodes.length;
			//first populate the Vector and the Hashtable with named nodes
			for(int i = 0; i < net_size; i++){
				Vector parent_list = new Vector();
				String node_name = v3nodes[i].getName();
				//get the values for a node
				Vector value_list = new Vector();
				int domain_order = v3nodes[i].getDomain().getOrder();
				for(int c = 0; c < domain_order; c++){
					value_list.add(v3nodes[i].getDomain().getName(c));
				}
			
				Vector unrolled_probs = unrollCPF(nodes[i].getCPF());
				NodeLifted oldstyle_node = new NodeLifted(oldstyle_net);
				oldstyle_node.setName(node_name);
				oldstyle_node.setProbabilities(unrolled_probs);
				//build the vector of values
				Vector values = new Vector();
				for(int g = 0; g < nodes[i].getDomain().getOrder(); g++){
					values.add(nodes[i].getDomain().getName(g));
				}
				oldstyle_node.setPossibleValues(values);
				nodestore.put(oldstyle_node.getName(), oldstyle_node);
				oldstyle_nodelist.add(oldstyle_node);
			}
			//now find the parents and the children, and set those
			for(int i = 0; i < v3nodes.length; i++){
				BeliefNode current = v3nodes[i];
				BeliefNode[] parents = new_net.getParents(current);
				BeliefNode[] children = new_net.getChildren(current);
				NodeLifted current_oldstyle = (NodeLifted)oldstyle_nodelist.get(i);
									
				for(int c = 0; c < children.length; c++){
					String name = children[c].getName();
					NodeLifted oldstyle_child = (NodeLifted)nodestore.get(name);
					current_oldstyle.addChild(oldstyle_child);
				}	
				oldstyle_net.addNode(current_oldstyle);
			}
			oldnodes = new NodeLifted[oldstyle_nodelist.size()];
			for(int z = 0; z < oldstyle_nodelist.size(); z++){
				oldnodes[z] = (NodeLifted) oldstyle_nodelist.get(z);
			}
			return oldstyle_net;
		}
		
	/*
	  * updateProbs updates the exact or approximate probabilities for all nodes
	  * @param current - a Vector of Vectors with the current probabilities
	  * @param sample - a new instantiation
	  * @param count - how many samples have now been produced
	  * @return Vector - the updated probabilities
	 */

	 private Vector updateProbs(Vector current, Vector sample, int count) {
		 Vector oneNodeProbs = new Vector();
		 Vector newProbs = new Vector();
		 //if Vector current is empty, fill it with 0's for all probs
		 if (current.size() == 0) {
			   for (int i = 0; i < network.size(); i++) {
					 Vector oneProb = new Vector();
					 for (int j = 0; j < network.getNodeAt(i).getArity(); j++) {
						   oneProb.addElement(0+"");
					 }
					 current.addElement(oneProb);
			   }
		 }

		 for (int i = 0; i < current.size(); i++) {
			 oneNodeProbs = (Vector) current.elementAt(i);
			 String instant = sample.elementAt(i).toString();
			 int position = 0;
			 for (int j = 0; j < network.size(); j++) {
				 NodeLifted n = (NodeLifted) network.getNodeAt(j);
				 for (int k = 0; k < n.getValues().size(); k++) {
					 if (instant.equals(n.getValues().elementAt(k).toString())) {
						   position = k;
					 }
				 }
			 }
			 Vector newOneProbs = new Vector();
			 for (int j = 0; j < oneNodeProbs.size(); j++) {
				 double prob = (new Double(oneNodeProbs.elementAt(j).toString())).doubleValue();
				 if (j == position) {
					 double numTimes = prob*(count - 1);
					 numTimes=numTimes+1;
					 newOneProbs.addElement((numTimes/count)+"");
				 }
				 else {
					 double numTimes = prob*(count - 1);
					 newOneProbs.addElement((numTimes/count)+"");
				 }
			 }
			 newProbs.addElement(newOneProbs);
		 }
		 return newProbs;
	 }

	
	 /**
	  * generateAllSamples generates m samples using AIS or SIS
	  */

	 public void generateAllSamples() {
	 	
	 	if(VC != null){
	 		VC.beginTransaction();
	 		VC.pushAndApplyOperator( new Annotation("Adaptive Importance Sampling - Intro"));
	 		VC.pushAndApplyOperator( new Delay("AIS_Introduction", 5000));
	 		//VC.pushAndApplyOperator(new CodePageSelect(-1));
	 		
	 		VC.pushAndApplyOperator(new CodePageSelect(9));
	 		//System.out.println("selecting line");
	 		VC.pushAndApplyOperator( new Annotation("Adaptive Importance Sampling - Intro"));
	 		VC.pushAndApplyOperator( new Delay("AIS_Introduction", 5000));
	 		VC.pushAndApplyOperator( new CodePageSelectLine(0));
	 		VC.pushAndApplyOperator(new CodePageSelectLine(1));
	 	}
	 	//AIS algorithm starts here
		 double WIscore = 0;
		 double totalweight = 0;
		 Vector approProbs = new Vector();
		 Vector tempApproProbs = new Vector();
		 probabilityArray = new Vector(network.size());

		 
			 eval.AIS = true;
		

		 int k = 0;
		 // do two heuristic initializations for AIS
		 	
		 	 if(VC != null){
		 	 	//VC.beginTransaction();
		 	 	VC.pushAndApplyOperator(new CodePageSelect(7));
		 	 	//VC.pushAndApplyOperator(new CodePageSelectLine(2)); 
		 	 	//VC.pushAndApplyOperator(new CodePageSelect(-1));
		 	 }
			 heuristicInitialize1();
			 if(VC != null){
			 	//VC.beginTransaction();
			 	VC.pushAndApplyOperator(new CodePageSelect(8));
			 	//VC.pushAndApplyOperator(new CodePageSelectLine(3));
			 	//VC.pushAndApplyOperator(new CodePageSelect(-1));
			 }
			 heuristicInitialize2();
			 if(VC != null){
			 	VC.pushAndApplyOperator(new CodePageSelect(9));
			 	VC.pushAndApplyOperator(new Annotation("Adaptive Importance Sampling - Sampling and Updating"));
			 	VC.pushAndApplyOperator(new NewColorLegend() );
			 	VC.pushAndApplyOperator(new ColorLegendMap(0, "inactive"));
			 	VC.pushAndApplyOperator(new ColorLegendMap(15, "scanning for evidence"));
			 	VC.pushAndApplyOperator(new ColorLegendMap(14, "evidence not found; adding to update list"));
			 	VC.pushAndApplyOperator(new ColorLegendMap(19, "instantiating"));
			 	VC.pushAndApplyOperator(new ColorLegendMap(20, "instantiated"));
			 }

		 nodechanges.clear();
		 boolean evfound = false;

		 if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(6));
		 //want to update ICPT for only nodes that are ancestors to evidence
		 for (int i = 0; i < network.size() && !evfound; i++) {
		 	 if(VC != null){
		 	 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("node", nodes[i].getName()));
		 	 	VC.pushAndApplyOperator(new VertexColor(nodes[i].getOwner(), 15));
		 	 	
		 	 }
			 evfound = nodes[i].hasEvidence();
			 
			 if (!evfound) {
			 	if(VC != null){
			 		VC.pushAndApplyOperator( new CodePageSelectLine(7));
			 	 	VC.pushAndApplyOperator( new CodePageSelectLine(8));
			 	 	VC.pushAndApplyOperator(new VertexColor(nodes[i].getOwner(), 16));
			 	 	
			 	}
				 nodechanges.addElement(network.getNodeAt(i).getName());
			 }
		 }

		 nodechanges = reordernodes(nodechanges);
		 Vector samples = new Vector();
		 Vector onesample = new Vector();
		 
		 if(VC != null){
		 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("num_samples", "" + m ));
		 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("interval", "" + l));	
		 }
		 for (int i = 0; i < m; i++){
		 	if(VC != null){
		 		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("i", "" + i ));
		 		VC.pushAndApplyOperator(new CodePageSelectLine(11));
		 		VC.pushAndApplyOperator(new CodePageSelectLine(12));
		 	}

			 if (i % l == 0 && i != 0){
				updateICPT(samples, tempApproProbs, k);
				if(VC != null){
					VC.pushAndApplyOperator(new CodePageSelectLine(27));
					VC.pushAndApplyOperator(new CodePageSelectLine(28));
					VC.pushAndApplyOperator(new CodePageSelectLine(30));
					VC.pushAndApplyOperator(new CodePageSelectLine(31));
				}
				 normalization();
				 samples.clear();
				 
				 k++;
				 changeProbs = true;
			 }
			
			 onesample = this.generateSample(eval, network, ICPT);
			 samples.addElement(onesample);
			 WIscore = eval.calculateSImpScore(onesample);
			 if(!evidence_specified) WIscore = 1; //for no evidence
			 changeProbs = false;
			 totalweight = totalweight + WIscore;
			 approProbs = eval.computeProbOfQueryNodesOntheFly(onesample, i+1, totalweight, "-f", WIscore);
			 tempApproProbs = this.parseProbs(approProbs, nodechanges);
			 if(VC != null){
			 	VC.pushAndApplyOperator(new Delay("sample_clear", 0));
			 	for(int z = 0; z < nodes.length; z++){
			 		VC.pushAndApplyOperator(new VertexColor(nodes[z].getOwner(), 0));
			 	}
			 	VC.pushAndApplyOperator(new Delay("done_clear", 500));
			 }
		 }
		 probabilityArray = approProbs;
		 //AIS algorithm ends here
		 if(VC != null){
		 	VC.pushAndApplyOperator(new CodePageSelectLine(45));
		 	VC.pushAndApplyOperator(new CodePageSelectLine(46));
		 	VC.pushAndApplyOperator(new CodePageSelectLine(47));
		 	VC.pushAndApplyOperator(new CodePageSelectLine(48));
		 	VC.pushAndApplyOperator(new CodePageSelectLine(49));
		 	VC.pushAndApplyOperator(new Annotation("AIS : done"));
		 	VC.pushAndApplyOperator(new CodePageSelect(-1));
		 	VC.commitTransaction();
		 }
	 }

	 /**
	  *  heuristicInitialize1 changes the ICPT tables to the parents of evidence nodes
	  *     to uniform distribution when P(E=e) for that evidence node < 1/(2*ne),
	  *     where ne is the number of outcomes possible for that evidence node.
	  *     Nodechanges is a vector of all nodes that are direct ancestors to evidence nodes.
	  */

	 private void heuristicInitialize1() {
	 //	System.out.println("Init 1");
		 nodechanges = new Vector();
		 Vector[] evandparents = new Vector[eval.getEvidenceNodes().size()];
		 Vector numofev = new Vector();

		 for (int j = 0; j < evandparents.length; j++){
			 evandparents[j]=new Vector();
		 }
		 
		 if(VC != null){
		 	VC.pushAndApplyOperator(new Annotation("Finding All Unlikely Evidence"));
		 	VC.pushAndApplyOperator(new NewColorLegend());
			VC.pushAndApplyOperator(new ColorLegendMap(0,"No activity"));
			VC.pushAndApplyOperator(new ColorLegendMap(14,"looking for evidence"));
			VC.pushAndApplyOperator(new ColorLegendMap(16,"unlikely evidence found"));
			VC.pushAndApplyOperator(new ColorLegendMap(17,"modifying these parents"));
			
		 }
		 int count = 0;
		 for (int j = 0; j < network.size(); j++){
		 	if(VC != null){
		 		
		 		VC.pushAndApplyOperator(new CodePageSelectLine(2));
		 		VC.pushAndApplyOperator(new VertexColor(nodes[j].getOwner(), 14 ));
		 		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("name", nodes[j].getName()));
		 		//VC.pushAndApplyOperator(new Delay("hi1_evscan", 5000));
		 		VC.pushAndApplyOperator(new VertexColor(nodes[j].getOwner(), 0));
		 		VC.pushAndApplyOperator(new CodePageSelectLine(3));
		 	}
			 if (nodes[j].hasEvidence()){
			 	if(VC != null) VC.pushAndApplyOperator(new VertexColor(nodes[j].getOwner(), 15));
				 numofev.addElement(new Integer(j));
				 for (int s = 0; s < network.getNodeAt(j).getParents().size(); s++){
				 	
					 evandparents[count].addElement(network.getNodeAt(j).getParents().elementAt(s));
					 String parent = network.getNodeAt(j).getParents().elementAt(s).toString();
					 
					 if(VC != null){
				 		VC.pushAndApplyOperator(new CodePageSelectLine(4));
				 		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("parent_name", parent));
				 		VC.pushAndApplyOperator(new CodePageSelectLine(6));
				 	}

					 if (!nodechanges.contains(parent) && eval.getQueryNodeNames().contains(parent)){
					 	if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(7));
						 nodechanges.addElement(parent);
					 }
				 }

				 count++;
			 }
	   }
	   nodechanges = reordernodes(nodechanges);
	   Vector evidence = eval.getEvidenceNodeNames();
	   
	   for (int i = 0; i<numofev.size(); i++){
	   		if(VC != null)VC.pushAndApplyOperator(new CodePageSelectLine(12));
	   			
	   		
		   Vector values = eval.getEvidenceValues();

          
		   count = 0;
		   double sum = 0.0;

		   //finds the index of the instantiated value of the current
		   //node in numofev (which hold the AllNodes indices of evidence values)
		   //e.g., a2 has index 1, a1 has index 0 (if a is an evidence node)
           //for (int s = 0; s < network.size(); s++) System.out.println(nodemanager.getValue(s));
		   count = eval.getValue(((Integer)numofev.elementAt(i)).intValue());
		   //interval is the size of the probabilitytable for the current evidence node / the number of possible values the node can take
		   int interval = this.network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getProbabilities().size() / network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getValues().size();
		   int start = interval * count;
		   //sums all the probabilities in the appropriate row (corresponding to the evidence node's
		   //instantiated value) in the probability table
		   for (int j = start; j<start + interval; j++){
		   	double oldsum = sum;
			   sum=sum+new Double(this.network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getProbabilities().elementAt(j).toString()).doubleValue();
			   if(VC != null){
			   	VC.pushAndApplyOperator(new CodePageSelectLine(13));
			   	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("sum", "" + sum));
			   	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("old_sum", "" + oldsum));
			   	VC.pushAndApplyOperator(new CodePageSelectLine(14));
			   }
		   }
		   //averages the probabilities
		   double average = sum / (this.network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getProbabilities().size()/2);
		   int numoutcomes = this.network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getValues().size();
		   if(VC != null){
		   	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("avg", "" + average));
		   	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("num_pos", "" + numoutcomes));
		   	VC.pushAndApplyOperator(new CodePageSelectLine(16));
		   	VC.pushAndApplyOperator(new CodePageSelectLine(17));
		   }
		   if (average < (1/(2*numoutcomes))){
		   	
			   for (int j = 0; j<evandparents[i].size(); j++){
			   	if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(18));
				   double newentry = 1/this.network.getNodeAt(new Integer(numofev.elementAt(i).toString()).intValue()).getProbabilities().size();
				   int index = network.indexOf(evandparents[i].elementAt(j).toString());
				   for (int z = 0; z < ((Vector) ICPT[index]).size(); z++){
				   	
					   NodeLifted temp = this.network.getNodeByName(evandparents[i].elementAt(j).toString());

					   int numofnode = 0;
					   boolean found = false;
					   for (int g = 0; g < network.size(); g++) {
						   if (network.getNodes().elementAt(g).equals(temp)) {
							   numofnode = g;
							   found = true;
							 
						   }
					   }
					   if (found) {
						   ICPT[numofnode].setElementAt((new Double(newentry)).toString(), z);
						   if(VC != null) VC.pushAndApplyOperator(new VertexColor(nodes[numofnode].getOwner(), 17));
					   }
					   if(VC != null){
					   	VC.pushAndApplyOperator(new CodePageSelectLine(19));
					   	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("this_node", nodes[index].getName()));
					   }
				   }

			   }
		   }
	   }
	   //normalizes ICPT after the changes
	   normalization();
	   if(VC != null){
	   	VC.pushAndApplyOperator(new CodePageSelectLine(24));
	   	//VC.pushAndApplyOperator(new CodePageSelect(-1));
	   //	VC.commitTransaction();
	   }
	 }

	 /**
	  * heuristicInitialize2 deals with extremely small probabilities.  If a node has
	  *    a probability table with an entry that is less than the set threshold,
	  *    0.04, then this probability p is replaced by (0.04)^2, and (0.04-p) is
	  *    subtracted from the largest probability IN THE SAME COLUMN of the probability table.
	  */

	 private void heuristicInitialize2() {
	 	//System.out.println("init 2");
	 	if(VC != null){
	 		VC.pushAndApplyOperator(new Annotation("AIS Stage Two - Find Small Probabilities"));
	 		VC.pushAndApplyOperator(new NewColorLegend());
	 		VC.pushAndApplyOperator(new ColorLegendMap(0, "no activity"));
	 		VC.pushAndApplyOperator(new ColorLegendMap(14, "checking node probs"));
	 		VC.pushAndApplyOperator(new ColorLegendMap(15, "resetting icpts in node"));
	 	}
		 for (int i = 0; i < this.network.size(); i++){
		 	if(VC != null){
		 		
		 		VC.pushAndApplyOperator(new CodePageSelectLine(1));
		 		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("node", nodes[i].getName()));
		 		VC.pushAndApplyOperator(new VertexColor(nodes[i].getOwner(), 14));
		 	}
			 double max = 0.0;
			 int maxindex = 0;
			 double p = 0.0;
			 boolean changed = false;

			 for (int j = 0; j < network.getNodeAt(i).getProbabilities().size(); j++){
			 	if(VC != null){
			 		VC.pushAndApplyOperator(new CodePageSelectLine(2));
			 		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("prob", network.getNodeAt(i).getProbabilities().elementAt(j).toString()));
			 		VC.pushAndApplyOperator(new CodePageSelectLine(3));
			 	}
			 	 boolean isEv = nodes[i].hasEvidence();
				 //determines if the current ICPT entry is less than the threshold, 0.04.
				 if ((new Double(network.getNodeAt(i).getProbabilities().elementAt(j).toString()).doubleValue() < 0.04) && !isEv){
					 p = new Double(network.getNodeAt(i).getProbabilities().elementAt(j).toString()).doubleValue();
					 ICPT[i].setElementAt((new Double(0.04*0.04)).toString(), j);
					 changed = true;
					 if(VC != null) {
					 	VC.pushAndApplyOperator(new VertexColor(nodes[i].getOwner(), 15));
					 	VC.pushAndApplyOperator(new CodePageSelectLine(4));
					 }
				 }
				 //if a change was made, make the aforementioned correction to the largest probability in the same
				 //column
				 if(VC != null)VC.pushAndApplyOperator(new CodePageSelectLine(6));
				 if (changed){
					 int numstates = network.getNodeAt(i).getArity();
					 int numrows = network.getNodeAt(i).getProbabilities().size()/numstates;
					 int start = (j%numrows);

					 for (int c = start; c < network.getNodeAt(i).getProbabilities().size(); c = c + numrows){
						 if (new Double(network.getNodeAt(i).getProbabilities().elementAt(c).toString()).doubleValue() > max){
							 max = new Double(network.getNodeAt(i).getProbabilities().elementAt(c).toString()).doubleValue();
							 maxindex = c;
						 }
					 }
					 
					 double entry = new Double(network.getNodeAt(i).getProbabilities().elementAt(maxindex).toString()).doubleValue();
					 ICPT[i].setElementAt((new Double(entry-(p-0.04))).toString(), maxindex);
					 if(VC != null){
					 	VC.pushAndApplyOperator(new CodePageSelectLine(7));
					 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("max_prob", "" + entry ));
					 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("small_prob", "" + p ));
					 	VC.pushAndApplyOperator(new CodePageSelectLine(8));
					 }
				 }

				 changed = false;
			 }
			 if(VC != null)VC.pushAndApplyOperator(new VertexColor(nodes[i].getOwner(), 0));
		 }

		 //normalizes ICPT table
		 normalization();
		 if(VC != null){
		 	VC.pushAndApplyOperator(new CodePageSelectLine(13));
		 	//VC.pushAndApplyOperator(new CodePageSelect(-1));
		 //VC.commitTransaction();
		 }
	 }

	 /**
	  * extractSamples parses the training data when using GASLEAK to get the exactprobs
	  * @param allSamples - a 2D string array of sample instantiations
	  */

	 private void extractSamples(String[][] allSamples) {
			 exactProbs.clear();
		 for (int i = 0; i < m; i++) {
				 Vector sample = new Vector();
				 for (int j = 0; j < network.size(); j++) {
					 sample.addElement(allSamples[i][j]);
				 }
				 exactProbs = this.updateProbs(this.exactProbs, sample, i+1);
		 }
	 }

	 /**
	  * parseProbs parses the probabilities that correspond to query nodes that are not
	  *    parents to evidence.
	  * @param probs - the original Vector of probabilities
	  * @param changes - a Vector of query nodes that are parents to evidence
	  * @return Vector - the parsed list of probabilities
	  */

	 private Vector parseProbs(Vector probs, Vector changes){
		 Vector newprobs = new Vector();

		 for (int i = 0; i < eval.getQueryNodeNames().size(); i++){
			 String name = eval.getQueryNodeNames().elementAt(i).toString();
			 boolean found = false;

			 for (int j = 0; j < changes.size() && !found; j++){
				 if (changes.elementAt(j).toString().equals(name)){
					 found = true;
				 }
			 }

			 if (found){
				 newprobs.addElement(probs.elementAt(i));
			 }
		 }
		 return newprobs;
	 }

	 /**
	  * updateICPT updates the ICPT table after a batch of samples
	  * @param currentsamples - l samples generated between update intervals
	  * @param probs - the approximate probilities for each query node
	  * @param k - the number of updates so far
	  */

	 private void updateICPT(Vector currentsamples, Vector ApproProbs, int k) {
		 Vector ChangingProb = new Vector();
		 Vector OneApproProb = new Vector();
		 Vector OneFreq = new Vector();
		 Vector AllFreq = new Vector();

		 for (int i = 0; i < eval.getFrequencies().size(); i++) {
			 AllFreq.addElement(eval.getFrequencies().elementAt(i));
		 }

		 AllFreq = this.parseExactFreqs(AllFreq, nodechanges);
		 String temp2 = "";
		 String temp1 = "";
		 NodeLifted node1 = new NodeLifted(network);
		 int counter = 0;
		 double weight = 0.0;
		 if(VC != null){
		 	VC.pushAndApplyOperator(new CodePageUpdateEnvironment("samples_so_far", "" + k));
		 	VC.pushAndApplyOperator(new CodePageSelectLine(13));
		 }
		 // update weight
		 if (k < m / l) {
		 	if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(14));
			 weight = 0.0;
		 }
		 else {
		 	if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(16));
			 weight = 1.0;
		 }

		 // update learning rate
		 double b = 0.14;
		 double a = 0.4;
		 double kmax = m/l;
		
		 //double learnrate = a*(Math.pow((b/a), k/kmax));
		 double learnrate = (kmax-k)/kmax;
		 if(VC != null){
		 	VC.pushAndApplyOperator( new CodePageUpdateEnvironment("kmax", "" + kmax));
		 	//VC.pushAndApplyOperator( new CodePageUpdateEnvironment("samples_so_far", "" + k));
		 	VC.pushAndApplyOperator( new CodePageSelectLine(18));
		 	
		 }
		 //double learnrate = Math.exp((-3.0*k)/(kmax+0.0));
		 for(int i =0;i<network.size();i++) {
		 	if( VC != null ){
		 		VC.pushAndApplyOperator( new CodePageUpdateEnvironment("node", nodes[i].getName()));
		 		VC.pushAndApplyOperator( new CodePageSelectLine(21));
		 	}
			 if(nodechanges.contains(network.getNodeAt(i).getName())) {
				 node1=(NodeLifted) network.getNodes().elementAt(i);

				 // Get the modifiable probability table.
				 ChangingProb = (Vector) ICPT[i];

				 temp1 = AllFreq.elementAt(counter).toString();
				 temp1 = temp1.substring(1,temp1.length()-1);
				 temp2 = ApproProbs.elementAt(counter).toString();
				 temp2 = temp2.substring(1,temp2.length()-1);

				 OneFreq.clear();
				 StringTokenizer t1 = new StringTokenizer(temp1,", ");
				 int count1 = t1.countTokens();

				 for(int j = 0; j < count1; j++) {
					 OneFreq.addElement(t1.nextToken());
				 }

				 OneApproProb.clear();
				 StringTokenizer t2 = new StringTokenizer(temp2,", ");
				 int count2 = t2.countTokens();

				 for(int j = 0; j < count2; j++) {
					 OneApproProb.addElement(t2.nextToken());}

				 int length = ChangingProb.size()/node1.getValues().size();

				 // Accessing the elements for every possible values of the current node.
				 int offSet = 0;

				 for(int j = 0; j < OneApproProb.size(); j++) {
				 	if (VC != null) VC.pushAndApplyOperator( new CodePageSelectLine(22));
				 		
				 
					 // Get the correction to be made.
					 double d1 = Double.parseDouble(OneFreq.elementAt(j).toString().trim());
					 double d2 = Double.parseDouble(OneApproProb.elementAt(j).toString().trim());

					 // Correction loop: for the current possible value.
					 for(int r = 0; r < length; r++) {
						 // Get the old probability value
						 double Prob = Double.parseDouble(ChangingProb.elementAt(r+offSet).toString().trim());

						 // Add the score for that node.
						 double newProb = 0;
						
							 newProb = Prob + learnrate*(d2 - Prob);
						
						 ChangingProb.setElementAt(String.valueOf(newProb),r+offSet);
						 if( VC != null){
						 	VC.pushAndApplyOperator( new CodePageUpdateEnvironment("old_prob", "" + Prob));
						 	VC.pushAndApplyOperator( new CodePageUpdateEnvironment("learnrate", "" + learnrate));
						 	VC.pushAndApplyOperator( new CodePageUpdateEnvironment("approx_prob", "" + d2));
						 	VC.pushAndApplyOperator( new CodePageSelectLine(23));
						 }
					 }

					 // for every poosible value, increment offSet, so that poniter points at the next set of values.
					 offSet = offSet + length;

				 }
				 
				 ICPT[i] = ChangingProb;
				 counter++;
			 }
		 }
	 }

	/**
	  * parseProbs parses the exactProbs to only contain those that correspond to query nodes
	  * @param Exact - the full list of exactProbs
	  * @return Vector - the parsed list of probabilities
	  */

	 private Vector parseExactProbs(Vector Exact)
	 {   Vector parseProbs = new Vector();
		 int count = 0;
		 for (int i = 0; i<network.size(); i++)
		 {   String name = network.getNodeAt(i).getName();
		 if (eval.getQueryNodeNames().contains(name)){
			   parseProbs.addElement(Exact.elementAt(count));
			 }
		 count++;
		 }
		 return parseProbs;
	 }

	 /**
	  * parseExactFreqs parses the frequencies that correspond to query nodes that are not
	  *    parents to evidence.
	  * @param probs - the original Vector of frequencies
	  * @param changes - a Vector of query nodes that are parents to evidence
	  * @return Vector - the parsed list of frequencies
	  */

	 private Vector parseExactFreqs(Vector Exact, Vector changes)
	 {   Vector parseProbs = new Vector();
		 for (int i = 0; i<eval.getQueryNodeNames().size(); i++)
		 {   String name = eval.getQueryNodeNames().elementAt(i).toString();
			 boolean found = false;
			 for (int j = 0; j<nodechanges.size() && !found; j++)
			 {   if (changes.elementAt(j).toString().equals(name))
				 {   found = true;}
			 }
			 if (found)
			 {   parseProbs.addElement(Exact.elementAt(i));}
		 }
		 return parseProbs;
	 }

	 /**
	   * normalization will normalize the updatable probability table (ICPT) for each
	   * Node in the network so that corresponding columns add to 1.
	   *
	   */

	  private void normalization() {
		 NodeLifted node1 = new NodeLifted(network);
		 Vector problist = new Vector();
		 Vector nodelist = network.getNodes();

		 //access each node in the nodelist
		 for(int i = 0; i < nodelist.size(); i++){
			 int numstates = ((NodeLifted) nodelist.elementAt(i)).getValues().size();
			 int numrows = ((NodeLifted) nodelist.elementAt(i)).getProbabilities().size()/numstates;
			 // Get the node.
			 node1 = (NodeLifted)network.getNodeAt(i);
			 problist = ICPT[i];

			 for (int j = 0; j < numrows; j++){
				 int start = (j%numrows);
				 double sum = 0;
				 double tempprob = 0;
				 Vector allprobs = new Vector();

				 for (int c = start; c < network.getNodeAt(i).getProbabilities().size(); c = c + numrows){
					 tempprob = new Double(problist.elementAt(c).toString()).doubleValue();
					 sum += tempprob;

					 allprobs.addElement((new Double(tempprob)).toString());
				 }

				 int count = 0;
				 for (int c = start; c < network.getNodeAt(i).getProbabilities().size(); c = c + numrows){
					 if (sum < .99999 || sum > 1.00001) {
						 tempprob = Double.parseDouble(allprobs.elementAt(count).toString()) / sum;
					 }
					 else {
						 tempprob = Double.parseDouble(allprobs.elementAt(count).toString());
					 }
					 problist.setElementAt(Double.toString(tempprob), c);
					 count++;
				 }
			 }
			 ICPT[i] = problist;
		 }
	  }

	  /**
	   * reordernodes puts the query nodes in order
	   * @param nodes - the current ordering of the query nodes
	   * @return Vector - the new ordering of query nodes
	   */

	 private Vector reordernodes(Vector nodes) {
		 Vector returnnodes = new Vector();

		 for (int i = 0; i < network.size(); i++) {
			 NodeLifted node1 = new NodeLifted(network);
			 node1=(NodeLifted) network.getNodes().elementAt(i);

			 if (nodes.contains(node1.getName())) {
				 returnnodes.addElement(node1.getName());
			 }
		 }
		 return returnnodes;
	 }
	
	 /**
	    * generateValue generates an instantiation of a node
	    * @param probIntervals - the intervals in the nodes's probabilitytable with
	    *      which the instantiation is selected
	    * @return int - the index of the generated value
	    */

	    private int generateValue(double probIntervals[])
	    {
	        int value = -1;
	        double f = generator.nextDouble();
	        int len = probIntervals.length - 1;
	        if (f == 0.0) return 0;

	        for (int i = 0; i < len; i++){
	            double lower = probIntervals[i];
	            double upper = probIntervals[i+1];
	            if ((f > lower) && (f <= upper)){
	                value = i;
	                break;
	            }
	        }
	        return value;
	    }
	    private int randomGenerate(int i, BBNLifted network, EvaluatorLifted nodemanager, Vector probs)
	    {
	      int value;
	      NodeLifted node = network.getNodeAt(i);
	      int arity = node.getArity();
	      Vector parents = node.getParents();
	      int numOfParents = parents.size();

	      double problist[] = new double[probs.size()];
	      double probIntervals[] = new double[arity + 1];
	      double templist[] = new double[arity + 1];
	      int parentvaluelist[] = new int[numOfParents];

	      for (int index = 0; index < probs.size(); index++){
	        problist[index] = (new Double(probs.elementAt(0).toString()).doubleValue());
	      }

	      double d = 0;
	      probIntervals[0] = 0;

	      for (int j = 0; j < numOfParents; j++){
	      	if(VC != null ){
	      		VC.pushAndApplyOperator(new CodePageSelectLine(36));
	      		VC.pushAndApplyOperator(new CodePageSelectLine(37));
	      	}
	        parentvaluelist[j] = nodemanager.getValue(network.indexOf((NodeLifted) parents.elementAt(j)));
	      }

	      int index = 1;
	      int num = 0;

	      int p_value = -1;

	      int whichColumn = 0;
	      int parentArity = 1;
	      for (int j = node.getParents().size() -  1; j >= 0; j--){
	        int chain = nodemanager.getValue(node.getParent(j));
	        parentArity *= node.getParent(j).getArity();

	        for (int k = node.getParents().size() - 1; k > j; k--){
	            chain *= node.getParent(k).getArity();
	        }
	        whichColumn += chain;
	      }

	      for (int j = 0; j < arity; j++){
	     
	      	Object[] obs =  node.getProbabilities().toArray();
	      
	      	Vector values = new Vector();
	      	for (int r = node.getParents().size() -  1; r >= 0; r--){
	      		values = node.getParent(r).getValues();   		
	      	}
	       //probIntervals[j+1] = probIntervals[j] + node.getProbability(whichColumn+parentArity*j);
	      	Double d_ob = (Double)(obs[whichColumn+parentArity*j]);
	      	probIntervals[j+1] = probIntervals[j] + d_ob.doubleValue();
	      }
	      if(VC != null){
	      	VC.pushAndApplyOperator(new CodePageSelectLine(39));
	      	VC.pushAndApplyOperator(new CodePageSelectLine(40));
	      	VC.pushAndApplyOperator(new CodePageSelectLine(41));
	      }
	      return generateValue(probIntervals);
	  }
	    public Vector generateSample(EvaluatorLifted nodemanager, BBNLifted network) {
	        Vector problist;

	        for(int r = 0; r < network.size(); r++) {  
	        	if(nodes[r].hasEvidence()){
	                String temp = network.getNodeAt(r).getStateName(nodemanager.getValue(r));
	                nodemanager.setValue(r, network.getNodeAt(r).getValues().indexOf(temp));
	            }
	            else{
	                problist = network.getNodeAt(r).getProbabilities();
	                nodemanager.setValue(r, randomGenerate(r, network, nodemanager, problist));
	            }
	        }
	        return nodemanager.reportSimpleVector();
	    }
	    /**
	     * generateSample generates a sample of the selected sampling type
	     * @param nodemanager - a copy of the NodeManager
	     * @param network - the Bayesian network
	     * @param ICPT - an array of vectors of the ICPT entries (for AIS and SIS only)
	     * @return Vector - the generated sample
	     */

	    public Vector generateSample(EvaluatorLifted nodemanager, BBNLifted network, Vector[] ICPT) {
	        Vector problist;
	        for(int r = 0; r < network.size(); r++) {
	        	if(VC != null){
	        		VC.pushAndApplyOperator(new CodePageSelectLine(31));
	        		VC.pushAndApplyOperator(new CodePageUpdateEnvironment("node", nodes[r].getName()));
	        		VC.pushAndApplyOperator(new CodePageSelectLine(32));
	        		VC.pushAndApplyOperator(new VertexColor(nodes[nodeIndex(network.getNodeAt(r).getName())].getOwner(), 19));
	        		VC.pushAndApplyOperator(new Delay("instan", 250));
	        		VC.pushAndApplyOperator(new VertexColor(nodes[nodeIndex(network.getNodeAt(r).getName())].getOwner(), 20));
	           	}
	        	if(nodes[r].hasEvidence()){
	                String temp = network.getNodeAt(r).getStateName(nodemanager.getValue(r));
	                nodemanager.setValue(r, network.getNodeAt(r).getValues().indexOf(temp));
	                if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(33));
	                
	            }
	            else{
	                problist = (Vector) ICPT[r];
	                if(VC != null) VC.pushAndApplyOperator(new CodePageSelectLine(35));
	                nodemanager.setValue(r, randomGenerate(r, network, nodemanager, problist));
	            }
	        }
	        return nodemanager.reportSimpleVector();
	    }

	
	   
	    /**
	     * inlist determines whether all the elements in parents are in finishedlist
	     * @param parents - the Vector of the current Node's parents
	     * @param finishedlist - a Vector of the current ordering of nodes
	     * @return boolean - whether all elements in parents are in finishedlist
	     */

	    private boolean inlist(Vector parents, Vector finishedlist){
	        boolean in = false;
	        int number = 0;

	        for(int i=0;i<parents.size();i++){
	            if(finishedlist.contains(parents.elementAt(i))){number++;}
	        }
	        if(number==parents.size()){in = true;}

	        return in;

	    }
	    
	    private void packageMarginals(){
	    	//System.out.println("Network Size: " + nodes.length);
	    	//System.out.println("Number of Marginals computed: " + probabilityArray.size());
	    	//hack alert:
	    	int num_ev = 0;
	    	HashMap ev_locs = new HashMap();
	    	for(int i = 0; i < nodes.length; i++){
	    		if(nodes[i].hasEvidence()){
	    			ev_locs.put(new Integer(i), "" + i);
	    			num_ev++;
	    		}
	    	}
	    	
	    	int adj = 0;
	    	for(int i = 0; i < nodes_formarginals.length; i++){
	    		//System.out.println("On scan ");
	    	//	System.out.println("Scanning along nodes");
	    		String name = nodes[i].getName();
	    		Domain domain = nodes[i].getDomain();
	    		BeliefNode new_node = new BeliefNode(name, domain);
	    		BeliefNode[] new_node1 = new BeliefNode[1];
	    		new_node1[0] = new_node;
	    		CPF marginal_cpf = new CPF(new_node1);
	    		//CPF marginal_cpf = nodes[i].getCPF();
	    		new_node.setCPF(marginal_cpf);
	    		if(nodes[i].hasEvidence()){
	    			
	    			Evidence e = nodes[i].getEvidence();
	    			new_node.setEvidence(e);
	    			nodes_formarginals[i] = new_node;
	    			
	    		}
	    		else{
	    		Vector marginals = (Vector)probabilityArray.get(adj);
	    		for(int c = 0; c < marginals.size(); c++){
	    			//System.out.println("Scanning along probs in " + new_node.getName());
	    			Double dobj = new Double((String)marginals.get(c));
	    			double d = dobj.doubleValue();
	    			ValueDouble prob = new ValueDouble(d);
	    			marginal_cpf.put(c, prob);
	    		}
	    		nodes_formarginals[i] = new_node;
	    		adj++;
	    		}
	    		
	    		
	    	}
	    }
	    private boolean checkNodesForEvidence(BeliefNode[] the_nodes){
	    	boolean ret = false;
	    	for(int i = 0; i < the_nodes.length; i++){
	    		if(the_nodes[i].hasEvidence()){
	    			ret = true;
	    		}
	    		else{}
	    	}
	    	return ret;
	    	
	    }
	    /** setNumSamples sets the number of samples to generate
	     * 
	     * @param samples - the number of samples
	     */
	    public void setNumSamples(int samples){
	    	m = samples;
	    }
	    /** setInterval sets the number of samples to make before
	     * the ICPTs are updates
	     * @param interval - the number of samples in an interval
	     */
	    public void setInterval(int interval){
	    	l = interval;
	    }
	    
	    private int nodeIndex(String name){
	    	int index = -1;
	    	for(int i = 0; i < nodes.length; i++){
	    		if(nodes[i].getName().equals(name)) index = i;
	    	}
	    	return index;
	    }

	    
}
