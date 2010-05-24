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
 * Created on Jul 22, 2004
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
 
package edu.ksu.cis.bnj.ver3.inference.approximate.sampling;

import java.util.*;
import java.io.*;
import edu.ksu.cis.bnj.ver3.core.*;


/**
 * @author aking
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class EvaluatorLifted {
	  private BBNLifted nodeslist;  // a copy of the bayesian network
	  private Vector samples; // all of the samples generated through Importance Sampling
	  private Vector AllQueryNodesFrequency;  // a vector that stores the generated
											  //frequencies for each state of all query nodes
	  //private NodeManager nodemanage; // a copy of the node manager
	  BeliefNetwork v3net; //this is the v3 net, has functionality that used to be in NodeManager
	  private int m;  // the total number of samples taken
	  private double[][] printingsarray; // stores the generated RMSE values
	  private String sampling_method;  // the type of sampling being done
	  private AIS SISandAIS;   // a copy of the sampling class
	  private Random generator = new Random(System.currentTimeMillis());
	  public boolean AIS = false;
	  public boolean SIS = false;
	  private Hashtable ht = new Hashtable();
	  private Hashtable simpHash = new Hashtable(100);
	  //private String evidence_filename = "";
	  //private String printingsFile = "printings.txt";
	  public Vector query_nodes_public;
	  public Vector query_node_names_public;
	  public Vector ev_nodes_public;
	  public Vector ev_node_names_public;
	  public Vector node_names_public;
	  private int nodeStates[];
	  
	public EvaluatorLifted(int totalsamples, BBNLifted network, BeliefNetwork bn , String s) {
		 m = totalsamples;
		 nodeslist = network;
		 v3net = bn;
		 sampling_method = s;
		// evidence_filename = ev;
		 printingsarray = new double[1][m];  // hard-coded to only one evidence line
		 AllQueryNodesFrequency = new Vector();
		 //NodeManager manage = new NodeManager(nodeslist);
		 //manage.openEvidenceFile(ev);
		query_nodes_public = getQueryNodes();
		query_node_names_public = getQueryNodeNames();
		ev_nodes_public = getEvidenceNodes();
		ev_node_names_public = getEvidenceNodeNames();
		//build the node states
		nodeStates = new int[network.size()];
		//for (int i = 0; i < network.size(); i++){
			
		//}
		 
	 }

	  /**
	  * Evaluator is the constructor and initializes variables
	  * @param totalsamples - the number of samples to be taken
	  * @param network - the Bayesian network
	  * @param nm - a copy of the NodeManager
	  * @param s - the type of sampling algorithm
	  * @param ev - the evidence file name for the network
	  * @param ais - a copy of the AdaptiveAndSelfIS class
	  */

	 public EvaluatorLifted(int totalsamples, BBNLifted network, BeliefNetwork bn, String s, AIS ais) {
		 m = totalsamples;
		 nodeslist = network;
		 v3net = bn;
		 sampling_method = s;
		 //evidence_filename = ev;
		 SISandAIS = ais;
		 printingsarray = new double[1][m];  // hard-coded to only one evidence line
		 AllQueryNodesFrequency = new Vector();
		 //NodeManager manage = new NodeManager(nodeslist);
		 //manage.openEvidenceFile(ev);
		query_nodes_public = getQueryNodes();
		query_node_names_public = getQueryNodeNames();
		ev_nodes_public = getEvidenceNodes();
		ev_node_names_public = getEvidenceNodeNames();
		nodeStates = new int[network.size()];
	 }

	 /**
	  * getFrequencies returns a vector of frequencies for all query nodes from
	  *  samples.
	  *
	  * @return Vector - vector of frequencies
	  */

	 public Vector getFrequencies() {
		 return AllQueryNodesFrequency;
	 }

	 /**
	  * clearFrequencies clears the vector of frequencies for all query nodes.
	  */
	 public void clearFrequencies() {
		 AllQueryNodesFrequency.clear();
	 }

	/**
	  * addRMSE adds a value to the RMSE array
	  * @param d - the new RMSE value
	  * @param row - the row in the array
	  * @param col - the column in the array
	  */

	 public void addRMSE(double d,int row, int col) {
		 printingsarray[row][col] = d;
	 }

	/**
	  * printResults prints the RMSE values in printingsarray
	  * @param printingsFile - the file to print the RMSE values to
	  */

	 public void printResults(String printingsFile) throws java.io.IOException {
	  // this.printingsFile = printingsFile;
	   printResults();
	 }

	 /**
	  * printResults prints the RMSE values in printingsarray
	  */

	 public void printResults() throws IOException{
		 PrintWriter outfile = new PrintWriter(new FileWriter("printings.txt"));
		 //PrintWriter outfile = new PrintWriter(new FileWriter("printings.txt"));
		 for (int i = 0; i < m; i++){

			 for (int j = 0; j < 1; j++){
				 outfile.print(printingsarray[j][i] + " ");
			 }

			 outfile.println();
		 }
		 outfile.close();
	 }

	 /**
	  * computeRMSE_DforOneFromTwoProbs computes the marginal RMSE for generated sample
	  * @param ExactProbsFromLS - a vector of the exact probabilities from LS
	  *     for the query nodes in the network
	  * @param ApproProbsFromSamples - a vector of the approximate probabilities
	  *     (based on samples taken) for the query nodes in the network.
	  * @return double - the RMSE value for the current sample
	  */

	 public double computeRMSE_DforOneFromTwoProbs(Vector ExactProbsFromLS, Vector ApproProbsFromSampling){
		 double rmse = 0;
		 Vector ExactProbs = ExactProbsFromLS;
		 Vector ApproProbs = ApproProbsFromSampling;
		 double N = 0;
		 String temp1 = "";
		 String temp2 = "";
		 Vector OneExactProb = new Vector();
		 Vector OneApproProb = new Vector();

		 for(int i = 0; i < ExactProbs.size(); i++){
			 temp1 = ExactProbs.elementAt(i).toString();
			 temp1 = temp1.substring(1,temp1.length()-1);
			 temp2 = ApproProbs.elementAt(i).toString();
			 temp2 = temp2.substring(1,temp2.length()-1);
			 OneExactProb.clear();
			 OneApproProb.clear();
			 StringTokenizer t1 = new StringTokenizer(temp1,", ");
			 StringTokenizer t2 = new StringTokenizer(temp2,", ");
			 int count1 = t1.countTokens();
			 int count2 = t2.countTokens();

			 for(int j = 0; j < count1; j++){
				 OneExactProb.addElement(t1.nextToken());
			 }

			 for(int j = 0; j < count2; j++){
				 OneApproProb.addElement(t2.nextToken());
			 }

			 for(int j = 0; j < OneApproProb.size(); j++){
				 N++;
				 double d1 = Double.parseDouble(OneExactProb.elementAt(j).toString().trim());
				 double d2 = Double.parseDouble(OneApproProb.elementAt(j).toString().trim());
				 rmse = rmse + (d1-d2)*(d1-d2);
			 }
		 }

		 rmse = Math.sqrt(rmse/N);

		 return rmse;

	 }

	 /**
	  * computeMPE estimates the MPE for the network
	  * @param allsamples - m samples generated by StochasticSampling
	  * @return Vector - the MPE
	  */

	 public Vector computeMPE(Vector allsamples){
		 Vector max = new Vector();
		 Vector temp = new Vector();
		 Object obj = new Object();
		 String temp1 = "";
		 int count = 0;

		 ht = new Hashtable();

		 for(int i = 0; i < allsamples.size(); i++){
			 temp1 = allsamples.elementAt(i).toString();

			 if(!ht.containsKey(temp1)){
				 ht.put(temp1, "1");
			 }
			 else if(ht.containsKey(temp1)){
				 count = Integer.parseInt(ht.get(temp1).toString());
				 count++;
				 ht.put(temp1, Integer.toString(count));
			 }

		}

		int hashtableEntries=0;
		for (Enumeration e = ht.elements() ; e.hasMoreElements() ;) {
		   e.nextElement();
		   hashtableEntries++;
		 }

		 Set S = ht.entrySet();

		 Object[] obj1 = S.toArray();
		 String[] hashtable = new String[hashtableEntries];

		 for(int i = 0; i < hashtableEntries; i++){
			 hashtable[i] = obj1[i].toString();
		 }

		 /*
		 data in String[] hashtable:
		 [a2, f2, b2, e2, c2, g1, d2, h1]=3
		 [a2, f2, b2, e2, c2, g1, d2, h2]=2
		 [a2, f1, b2, e2, c2, g1, d2, h1]=4
		 [a2, f1, b2, e2, c2, g1, d2, h2]=1
		 */

		 int maxIndex = 0;
		 int tempFreq = 0;
		 int maxFreq = 0;

		 for(int i = 0; i < hashtableEntries; i++){
			 tempFreq = Integer.parseInt(hashtable[i].substring(hashtable[i].indexOf("=")+1));

			 if(tempFreq > maxFreq){
				 maxFreq = tempFreq;
				 maxIndex = i;
			 }

		 }

//		   System.out.println("MPE--"+hashtable[maxIndex]);

		 String MPE = hashtable[maxIndex].substring(1,hashtable[maxIndex].lastIndexOf("]")).trim();
		 StringTokenizer t1 = new StringTokenizer(MPE,", ");
		 int count1 = t1.countTokens();

		 for(int j = 0; j <count1; j++){
			 max.addElement(t1.nextToken());
		 }

		 return max;
	 }

	 /**
	  * getConditionalProbOfE returns the probability of evidence in the sample
	  * @param i - the place of the evidence node in the whole network
	  * @param evidencValue - the instantiation of the evidence node
	  * @param OneSample - the generated sample
	  * @return double - the probability of evidence in the sample
	  */
	 private double getConditionalProbOfE(int i, String evidencValue, Vector OneSample){
		 double conditionalProb = 0;
		 int index =0;
		 int p = nodeslist.getNodeAt(i).getParents().size();
		 Vector problist = new Vector();
		 Vector parentnamelist = new Vector();
		 Vector parentvaluelist = new Vector();
		 problist = nodeslist.getNodeAt(i).getProbabilities();
//	   System.out.println("Node name is " + nodeslist.getNodeAt(i).getName() + " and its values are: ");
//	   nodeslist.getNodeAt(i).printStateNames();

		 if(p == 0){
			 index = nodeslist.getNodeAt(i).getValues().indexOf(evidencValue);



		 }
		 else if(p > 0){
			 int temp_index1 = nodeslist.getNodeAt(i).getValues().indexOf(evidencValue);

//		   System.out.println("temp_index1 = " + temp_index1);
			 parentnamelist = nodeslist.getNodeAt(i).getParentNames();
			 for(int j=0;j<parentnamelist.size();j++){
				  int nodeIndex = this.nodeslist.indexOf(parentnamelist.elementAt(j).toString());
				  parentvaluelist.addElement(OneSample.elementAt(nodeIndex).toString());
			 }

		   // now both  parentnamelist and parentvaluelist are ready
			 int temp_index2 = 0;
			 int num = 0;
			 String p_name = "";
			 String p_value = "";
			 Vector p_values = new Vector();
			 int length = problist.size()/(nodeslist.getNodeAt(i).getValues().size());
			 int sizeweight = length;

			 for(int j = 0; j < parentnamelist.size(); j++){
				 p_name = parentnamelist.elementAt(j).toString();
				 p_value = parentvaluelist.elementAt(j).toString();
				 p_values = nodeslist.getNodeByName(p_name).getValues();
				 sizeweight = sizeweight/nodeslist.getNodeByName(p_name).getValues().size();
				 num = p_values.indexOf(p_value);
				 temp_index2 = temp_index2 + num*sizeweight;
			 }

			 index = length*temp_index1 + temp_index2;
		 }
		 //if (index < 1) {index = 0;}
		 conditionalProb =Double.parseDouble(problist.elementAt(index).toString());
		 return  conditionalProb;
	 }

	 /**
	  * calculateSImpScore calculates the score of a given sample
	  * @param samplePoint - the generated sample
	  * @return double - the calculated score
	  */

	 public double calculateSImpScore(Vector samplePoint){
		 if (!SISandAIS.changeProbs && this.simpHash.containsKey(samplePoint.toString())) {
			 return ((new Double(this.simpHash.get(samplePoint.toString()).toString())).doubleValue());
		 }
		 double num = this.numerator(samplePoint);
		 double den = this.denominator(samplePoint);
		 if(den > 0){
			 simpHash.put(samplePoint.toString(), (num/den) + "");
			 return (num / den);
		 }
		 return 0;
	 }

	 /**
	  * numerator calculates the numerator of the formula for computing the score
	  * @param OneSample - the generated sample
	  * @return double - the numerator in the formula
	  */

	 private double numerator(Vector OneSample){
		 double liklihood = 1.0;
		 String str = "";

		 for(int i = 0; i < OneSample.size(); i++){
			 str = OneSample.elementAt(i).toString();

//		   System.out.println("About to check i = " + i + " with str = "  + str);
			 double temp = this.getConditionalProbOfE(i,str,OneSample);
			 liklihood = liklihood*temp;
		 }
		 return liklihood;
	 }

	 /**
	  * denominator calculates the denominator of the formula for computing the score
	  * @param OneSample - the generated sample
	  * @return double - the denominator in the formula
	  */

	 private double denominator(Vector OneSample){
		 double liklihood = 1.0;
		 String str = "";

		 // For all the nodes in the network
		 for(int i = 0; i < OneSample.size(); i++){
			 // Check whether the current node is query node or not.
			 //if(!this.nodemanage.isEvidence(i)){ //change to query the BeliefNet instead
			 if(!v3net.getNodes()[i].hasEvidence()){ //changed by aking
				 str = OneSample.elementAt(i).toString();
				 liklihood = liklihood*(this.getConditionalProbOfQ(i,str,OneSample));
			 }
		 }
		 return liklihood;
	 }

	 /**
	  * getConditionalProbOfE calculates the product of probabilities for query nodes
	  * @param i - the place of the node in the network
	  * @param evidencValue - the instantiation of the current query node
	  * @param OneSample - the generated sample
	  * @return double - the product of probabilities
	  */

	 private double getConditionalProbOfQ(int i, String evidencValue, Vector OneSample){
		 double conditionalProb =0;
		 int index =0;
		 int p = nodeslist.getNodeAt(i).getParents().size();
		 Vector problist = new Vector();
		 Vector parentnamelist = new Vector();
		 Vector parentvaluelist = new Vector();

		 // Get the changing probability table, which is required in denomintor part.
		 problist = (Vector) SISandAIS.ICPT[i];

		 // No parents (root node)
		 if(p == 0) {
			 index = nodeslist.getNodeAt(i).getValues().indexOf(evidencValue);
		 }
		 //for all nodes other than root nodes
		 else if(p > 0) {
			 int temp_index1 = nodeslist.getNodeAt(i).getStateIndex(evidencValue);


			 // This list contains name of all parent nodes.
			 parentnamelist = nodeslist.getNodeAt(i).getParentNames();
			 for(int j = 0; j < parentnamelist.size(); j++){
				 int nodeIndex = nodeslist.indexOf(parentnamelist.elementAt(j).toString());

			  // This list contains the values of the parent nodes for the given sample.
				  parentvaluelist.addElement(OneSample.elementAt(nodeIndex).toString());
			 }
			 // now both  parentnamelist and parentvaluelist are ready

			 int temp_index2 = 0;
			 int num = 0;
			 String p_name = "";
			 String p_value = "";
			 Vector p_values = new Vector();

			 // Here we are calculating how many elements in the probability list are for each
			 // possible value of the node.
			 // for example: node 'a' is parent of node 'c' and possible values of 'c' are
			 // c1 and c2 and that of a are a1 and a2. Size of prob. table will be 4.
			 // and thus length will be 4/2 =2 which means that for each possible value of 'c'
			 // prob table has 2 elements. (c1|a1, c1|a2).
			 int length = problist.size()/(nodeslist.getNodeAt(i).getValues().size());
			 int sizeweight = length;

			 for(int j = 0; j < parentnamelist.size(); j++){
				 p_name = parentnamelist.elementAt(j).toString();
				 p_value = parentvaluelist.elementAt(j).toString();

			 // Possible values of parent node(a1,a2)
				 p_values = nodeslist.getNodeByName(p_name).getValues();

			 // Total values in prob table which correspond to given evidence value/size of possible values of parent
				 sizeweight = sizeweight/nodeslist.getNodeByName(p_name).getValues().size();
				 num = p_values.indexOf(p_value);
				 temp_index2 = temp_index2 + num*sizeweight;
			 }

			 index = length*temp_index1 + temp_index2;
		 }
		 if (index < 0) {index = 0;}
		 conditionalProb = Double.parseDouble(problist.elementAt(index).toString());
		 return  conditionalProb;
	 }

	 /**
	  * computeProbOfQueryNodesOntheFly computes the probability of each instantiation
	  *    of each query node based on the samples thus far.
	  * @param SamplePoint - the current sample
	  * @param t - the number of samples that have been generated so far
	  * @param totalweight - the sum of the weights that each sample has produced
	  *    so far
	  * @return Vector - the Vector of probabilities for each instantiation of each
	  *    query node
	  */
//	  a lot of string operations can be trimmed here
	 public Vector computeProbOfQueryNodesOntheFly(Vector SamplePoint, int t, double totalweight, String sampling_method, double oneweight){

		 int numQuery = query_nodes_public.size(); //changed by aking
		 boolean AISnoEv = false;
		 if (AIS) {
			 if (!SISandAIS.evidence_specified) {
				 numQuery = nodeslist.size();
				 AISnoEv = true;
			 }
		 }

		 double samplesize = t;
		 // if Frequency table is null, initialize both query nodes' Frequency table and exact probs table to all "0"
		 Vector AllQueryNodesProbs = new Vector();
		 if(this.AllQueryNodesFrequency.isEmpty()){
			 for(int i=0;i<numQuery;i++){
				 Vector OneQueryNodeFrequency = new Vector();
				 String nodename = null;
				 if (AISnoEv == false) {
					 nodename = query_node_names_public.elementAt(i).toString(); //changed by aking
				 }
				 else {
					 nodename = nodeslist.getNodeAt(i).getName();
				 }

				 //int index = nodemanage.getNodeNames().indexOf(nodename);
				 for(int j = 0; j < this.nodeslist.getNodeByName(nodename).getArity();j++){
					 OneQueryNodeFrequency.addElement("0");
				 }
				 this.AllQueryNodesFrequency.addElement(OneQueryNodeFrequency);
			 }
		 }
		 double likelihood = oneweight;

		 Vector tempVector = new Vector();
		 double prob = 0;
		 double freq = 0;
		 String temp = "";

		 for(int i = 0; i < numQuery; i++){
			 NodeLifted node = null;
			 if (AISnoEv == false) {
				 node = (NodeLifted) query_nodes_public.elementAt(i); //changed by aking
			 }
			 else {
				 node = (NodeLifted) nodeslist.getNodeAt(i);
			 }
			 int index = nodeslist.indexOf(node);
			 Vector OneQueryNodeFrequency = new Vector();
			 OneQueryNodeFrequency = (Vector) AllQueryNodesFrequency.elementAt(i);

			 for(int j = 0; j < node.getArity(); j++){
				 String nodevalue = node.getValues().elementAt(j).toString().trim();
				 if(SamplePoint.elementAt(index).toString().trim().equals(nodevalue)){
					 freq = Double.parseDouble(OneQueryNodeFrequency.elementAt(j).toString());
					 if(sampling_method.equals("-f")){
						 freq = freq +likelihood;
					 }
					 else if(sampling_method.equals("-l")){
						 freq = freq +1;
					 }
					 OneQueryNodeFrequency.setElementAt(freq+"",j);
				 }
			 }
			 this.AllQueryNodesFrequency.setElementAt(OneQueryNodeFrequency, i);
		 }
		 // compute AllQueryNodesProbs from AllQueryNodesFrequency
		 for(int i = 0; i < AllQueryNodesFrequency.size(); i++){
			 Vector OneQueryNodesProb = new Vector();
			 tempVector = (Vector)AllQueryNodesFrequency.elementAt(i);
			 for(int j = 0; j < tempVector.size(); j++){
				 prob = Double.parseDouble(tempVector.elementAt(j).toString());

			   if(totalweight > 0){
					 prob = prob/totalweight;
			   }
			   else{
				   prob = 0;
			   }
			   OneQueryNodesProb.addElement(String.valueOf(prob));
			 }
			 AllQueryNodesProbs.addElement(OneQueryNodesProb);
		 }
		 return AllQueryNodesProbs;
	 }

	 /**
	  * clearFreqTable clears the frenquency table for each query node
	  */

	 public void clearFreqTable(){
		 this.AllQueryNodesFrequency.clear();

		 for(int i = 0; i < query_node_names_public.size();i++){ //changed by aking
			 Vector OneQueryNodeFrequency = new Vector();
			 String nodename = query_node_names_public.elementAt(i).toString(); //changed by aking
			 int index = node_names_public.indexOf(nodename);

			 for(int j = 0; j < nodeslist.getNodeByName(nodename).getValues().size(); j++){
				 OneQueryNodeFrequency.addElement("0");
			 }
			 this.AllQueryNodesFrequency.addElement(OneQueryNodeFrequency);
		 }
	 }

	 /**
	  * getLikelihoodOfSample computes the likelihood (probability) of the currently
	  *    generated sample
	  * @param OneSample - the current sample
	  * @return double - the computed likelihood of that sample
	  */

	 public double getLikelihoodOfSample(Vector OneSample){
		 if (this.simpHash.containsKey(OneSample.toString())) {
			 return ((new Double(this.simpHash.get(OneSample.toString()).toString())).doubleValue());
		 }
		 double liklihood = 1.0;
		 String str = "";
		 //int place = 0;
		 for(int i = 0; i < this.nodeslist.size(); i++){
			 if(v3net.getNodes()[i].hasEvidence()){ //
				 str = OneSample.elementAt(i).toString();
				 liklihood = liklihood*(this.getConditionalProbOfE(i,str,OneSample));
				 //place++;
			 }
		 }
		 simpHash.put(OneSample.toString(), liklihood + "");
		 return liklihood;
	 }
	
	//TODO: cache some of these values so we don't have to make unecessary loops
	public Vector getQueryNodes(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector query_nodes = new Vector(net_size);
		for(int i = 0; i < net_size; i++){
			if(!nodes[i].hasEvidence()) query_nodes.add(nodeslist.getNodeAt(i));
		}
		return query_nodes;
	}
	//TODO: cache some of these values so we don't have to make unecessary loops
	public Vector getQueryNodeNames(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector query_node_names = new Vector(net_size);
		for(int i = 0; i < net_size; i++){
			if(!nodes[i].hasEvidence()) query_node_names.add(nodeslist.getNodeAt(i).getName()); 
		}
		return query_node_names;
	}
	
	public Vector getEvidenceNodes(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector ev_nodes = new Vector(net_size);
		for(int i = 0; i < net_size; i++){
			if(nodes[i].hasEvidence()) ev_nodes.add(nodeslist.getNodeAt(i));
		}
		return ev_nodes;
	}
	public Vector getEvidenceValues(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector ev_vals = new Vector();
		for(int i = 0; i < net_size; i++){
			if(nodes[i].hasEvidence()){
				Evidence e = nodes[i].getEvidence();}}return ev_vals;}
				
					
	public Vector getEvidenceNodeNames(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector ev_node_names = new Vector(net_size);
		for(int i = 0; i < net_size; i++){
			if(nodes[i].hasEvidence()) ev_node_names.add(nodeslist.getNodeAt(i).getName());
		}
		return ev_node_names;
	}
	
	public Vector getNodeNames(){
		BeliefNode[] nodes = v3net.getNodes();
		int net_size = nodes.length;
		Vector node_names = new Vector(net_size);
		for(int i = 0; i < net_size; i++){
			node_names.add(nodeslist.getNodeAt(i).getName());
		}
		return node_names;
	}
	
	public void setValue(int node_index, int node_state){
		nodeStates[node_index] = node_state;
	}
	
	public int getValue(int index){
		return nodeStates[index];
	}
	
	public int getValue(NodeLifted node){
		return getValue(nodeslist.indexOf(node));
	}
	
	public Vector reportSimpleVector(){
		Vector vect = new Vector();
		//System.out.println(nodeslist.size());
		for (int i = 0; i < nodeslist.size(); i++){
		      vect.addElement(nodeslist.getNodeAt(i).getStateName(getValue(i)));
		    }
		    return vect;
	}

	
}
