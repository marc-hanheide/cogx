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

/**
 * @author aking
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class NodeLifted {
	private BBNLifted network;  // the bayesian network
	  private String type;  // ??
	  private String nodeName;  // the node's name

	  private Vector possibleValueList;  // a vector of strings containing the
										 // possible values of a node

	  private Vector parentList; // the vector of Node objects that are parents of
								 // this node
	  private Vector parentNames;  // the Vector of String objects that are
								   // parents to this node

	  private Vector childrenList;  // the vector of Node objects taht are children
									// of this node.

	  private Vector probabilityTable;  // the vector of doubles that correspond
										  // to the probabilities of the current
										  // node.
	  private Hashtable parentstore = new Hashtable();
	  private Hashtable childstore = new Hashtable();

	  private String genedata;  // the current instantiation of the node
	  private Vector varvalues;

	  private double x;  // node's x position
	  private double y;  // node's y position
	  private boolean process = false;
	  private Vector frequencytable = new Vector(); // for forward sampling, to compute the RMSE
	  public int pathLength = 0;

	  /**
	   * Node constructs the node and initializes the values.
	   * @param newNetwork - the node's parent Bayesian Network.
	   */
	  public NodeLifted(BBNLifted newNetwork) {
		network = newNetwork;
		nodeName = new String("Noname");
		possibleValueList = new Vector();
		parentList = new Vector();
		parentNames = new Vector();
		childrenList = new Vector();
		probabilityTable = new Vector();
		varvalues = new Vector();
		genedata = new String();
		x = y = 0.0;
	  }

		/**
	   * Creates a copy of the node.  Note that the parents and children are not
	   * cloned; they are simply re-referenced.  If you want a distinct set, you
	   * will need to copy all of the parents and all of the children.  Lucky for
	   * you, BBN.copy() does this for you; the Node.copy(BBN) should only be
	   * called if you intend to manipulate the probability table.  Note that neither
	   * the parents nor the children know about this copy.  It is recommended to
	   * use BBN.copy over Node.copy.
	   *
	   * @param newNetwork - the network this copy will belong to
	   * @return Node - the new copy.
	   * @see BBN.copy
	   *
	   */
	  public NodeLifted copy(BBNLifted newNetwork){
		NodeLifted newNode = new NodeLifted(newNetwork);
		Vector probabilities = new Vector(probabilityTable.size());

		for (int i = 0; i < probabilityTable.size(); i++){
		  probabilities.add(new Double(((Double) probabilityTable.elementAt(i)).doubleValue()));
		}
		newNode.setProbabilities(probabilities);
		newNode.setParentList((Vector) parentList.clone());
		newNode.setParentNames((Vector) parentNames.clone());
		newNode.setChildrenList((Vector) childrenList.clone());
		newNode.setName(new String(getName()));

		Vector values = new Vector(possibleValueList.size());
		for (int i = 0; i < possibleValueList.size(); i++){
		  values.add(new String((String) possibleValueList.elementAt(i)));
		}

		newNode.setPossibleValues(values);

		newNode.setX(x);
		newNode.setY(y);


		return newNode;
	  }

	  /**
	   * getArity returns the arity (number of possible values) of the node.
	   * @return int - the arity of the network
	   */
	  public int getArity(){
		return possibleValueList.size();
	  }

	  public void setPossibleValues(Vector values){
		possibleValueList = values;
	  }

	  public void setChildrenList(Vector children){
		childrenList = children;
	  }

	  public void setParentList(Vector parents){
		parentList = parents;
	  }

	  public void setParentNames(Vector names) {
		parentNames = names;
	  }

	  public void setProbability(int index, Double value){
		this.probabilityTable.set(index, value);
	  }

	  public void setValue(int index, Double value){
		varvalues.set(index, value);
	  }

	  public Double getValue(int index){
		return (Double) varvalues.get(index);
	  }

	  /**
	   * addProbability adds a new probability to the probability vector.
	   * @param newProbability - the new probability to add.
	   */
	  public void addProbability(Double newProbability){
		probabilityTable.add(newProbability);
	  }

	  /**
	   * getStateIndex matches a given string to the location in the possible value
	   * list.
	   *
	   * @param string - the string being matched
	   * @return int - the index of the string.
	   */
	  public int getStateIndex(String string) {
		for (int i = 0; i < possibleValueList.size(); i++){
		  if (((String) possibleValueList.elementAt(i)).equals(string)){
			return i;
		  }
		}
		return 0; //not correct :)

	  }

	  public void printStateNames(){
		System.out.println("For node " + getName() + ":");
		for (int i = 0; i < possibleValueList.size(); i++){
			System.out.println("  " + (String) possibleValueList.elementAt(i));
		}
	  }

	  /**
	   * print prints the name of the node.
	   */
	  public void print(){
		//Debug.print(getName());
	  }

	  /**
	   * setInstant will set the current instantiation to the passed string.
	   * @param instant - the new instantiation
	   *
	   * @deprecated The whole point of NodeManager is to handle instantiations.  It
	   * handles it with integers rather than strings, which makes projects run much
	   * faster.
	   */

	  public void setInstant(String instant) {
		genedata = instant;
	  }

	  /**
	   * getInstant will return the current instantiation of the node
	   * @return String - the current instantiation
	   *
	   * @deprecated The whole point of NodeManager is to handle instantiations.  It
	   * handles it with integers rather than strings, which makes projects run much
	   * faster.
	   */
	  public String getInstant() {
		return genedata;
	  }

	  /**
	   * setProbabilities will set the probability vector to the passed vector.
	   * @param newProbabilities - the new probabilities vector
	   */
	  public void setProbabilities(Vector newProbabilities){
		probabilityTable = newProbabilities;
	  }

	  /**
	   * getStateName will return the requested state index's name that was given
	   * in the network file.
	   *
	   * @param index - the index of the state being requested.
	   * @return String - the name of the state
	   */
	  public String getStateName(int index){
		return (String) possibleValueList.elementAt(index);
	  }

	  /**
	   * getStateNames will return all possible state names.
	   *
	   * @return Vector - the vector of strings pertaining to the possible values.
	   */
	  public Vector getStateNames(){
		return possibleValueList;
	  }

	  /**
	   * getNetwork returns the host bayesian network.
	   * @return BBN - the network.
	   */
	  public BBNLifted getNetwork(){
		return network;
	  }

	  /**
	   * getParent returns the parent at location index.  Throws BBNE_OutOfBounds
	   * exception if index is out of bounds.
	   *
	   * @param index - the requested parent index
	   * @return Node - the parent corresponding to the index
	   */
	  public NodeLifted getParent(int index) {
		//try{
		  return (NodeLifted) parentList.elementAt(index);
		//}
		//catch (java.lang.ArrayIndexOutOfBoundsException e){
		 // throw new BBNE_OutOfBounds();
		//}

	  }

	  public String getParentName(int index) {
		return ((NodeLifted) parentList.elementAt(index)).getName();

	  }

	  /**
	   * getParents returns the vector of Node objects containing the node's parents
	   *
	   * @return Vector - the parent vector
	   */
	  public Vector getParents(){
		return parentList;
	  }

	  public Vector getParentNames(){
		return parentNames;
	  }

	  public void empty_parentlist() {
		  this.parentList.removeAllElements();
		  this.parentNames.removeAllElements();
	  }

	  /**
	   * getChild returns the child at location index.  Throws BBNE_OutOfBounds
	   * exception if index is out of bounds.
	   *
	   * @param index - the requested child index
	   * @return Node - the child corresponding to the index
	   */
	  public NodeLifted getChild(int index) {
	   //try{
		  return (NodeLifted) childrenList.elementAt(index);
		//}
		//catch(java.lang.IndexOutOfBoundsException e){
		  //throw new BBNE_OutOfBounds();
		//}
	  }

	  /**
	   * getChildren returns the vector of Node objects containing the node's
	   * children
	   *
	   * @return Vector - the children vector
	   */
	  public Vector getChildren(){
		return childrenList;
	  }

	 /**
	   * getChildrenNames returns the vector of Strings containing the node's
	   * children's names.  Please be aware that getChildren() is much more useful.
	   *
	   * @return Vector - the vector of all children names
	   */
	  public Vector getChildrenNames(){
		Vector childrenNames = new Vector();
		for(int i=0;i<childrenList.size();i++){
		  childrenNames.addElement(getChild(i).getName());
		}
		return childrenNames;
	  }

	  /**
	   * getProbability returns the probability at location index.  Throws
	   * BBNE_OutOfBounds exception if index is out of bounds.
	   *
	   * @param index - the requested probability index
	   * @return double - the probability corresponding to the index
	   */
	  public double getProbability(int index) {
		//try{
		  return ((Double) probabilityTable.elementAt(index  )).doubleValue();
		//}
		//catch(java.lang.IndexOutOfBoundsException e){
		  //throw new BBNE_OutOfBounds();
		//}
	  }

	  public double getProbability(int column, int row){
		return 0.0;
	  }


	  /**
	   * getProbabilities returns the vector of Node objects containing the node's
	   * probabilities
	   *
	   * @return Vector - the probabilities vector
	   */
	  public Vector getProbabilities(){
		return probabilityTable;
	  }

	  /**
	   * getValues returns the vector of all possible instantiations of the node
	   * @return Vector - the vector of all instantiations
	   */

	  public Vector getValues() {
		return possibleValueList;
	  }

	  /**
	   * getName returns the name of the node.
	   * @return String - the name of the node.
	   */
	  public String getName(){
		return nodeName;
	  }

	  /**
	   * setName sets the name of the node.
	   * @param name - the new name.
	   */
	  public void setName(String name){
		nodeName = name;
	  }

	  /**
	   * setX sets the X coordinate
	   * @param x - the X coordinate
	   */
	  public void setX(double x){
		this.x = x;
	  }

	  /**
	   * getX returns the X coordinate.
	   * @return x - the x coordinate.
	   */
	  public double getX(){
		return this.x;
	  }

	  /**
	   * addValue adds a new value to the set of values the node can have.
	   * @param value - the new value
	   */
	  public void addValue(String value){
		possibleValueList.add(value);
	  }

	  /**
	   * addChild adds a child to the node.  calls addParent on the child
	   * @param child - the child being added to the node.
	   */
	  public void addChild(NodeLifted child){
		//getChildren().add(child);
	  
		childrenList.addElement(child);
		child.addParent(this);
	  
	  
	  }

	  /**
	   * addParent adds a parent to the node.  Caution - do not use this function
	   * in conjuction with addChild because addChild does this for you
	   * automatically.
	   */
	  public void addParent(NodeLifted parent){
		//getParents().add(parent);
		//getParentNames().add(parent.getName());
		parentList.addElement(parent);
		parentNames.addElement(parent.getName());
	  }

	  /**
	   * setY sets the Y coordinate.
	   * @param y - the y coordinate.
	   */
	  public void setY(double y){
		this.y = y;
	  }

	 /**
	   * getY returns the Y coordinate.
	   * @return y - the y coordinate.
	   */
	  public double getY(){
		return this.y;
	  }

	  /**
	   * numParents returns the number of parents this node has.
	   * @return int - the number of parents.
	   */
	  public int numParents(){
		return parentList.size();
	  }

	  /**
	   * numChildren returns the number of children this node has
	   * @return int - the number of children.
	   */
	  public int numChildren(){
		return childrenList.size();
	  }

	  public void labelProcessed() {
		  process = true;
	  }

	  public boolean isProcessed(){
		  return process;
	  }

	  public Vector getTable(int splitTableIndex, BBNLifted nodesGroup) {
		  int oldTableSize=probabilityTable.size();
		  //get oldTableSize e.g. it is 16 and following all use this number
		  int NodeValueNumber=this.getArity();

		  double[] normNumber = new double[NodeValueNumber];//this is array because there are three nomalization number
		  Vector[] subtable = new Vector[NodeValueNumber];//this is vector of vector
		  int splitrange = oldTableSize/NodeValueNumber;
		  //size of subtable
		  int accumulationNumber=1;//accumulationNumber is initiated as 1
		  int jumpnumber=1;//jumpNumber is initiated as 1
		  Vector putbackprobability=new Vector();//construct a vector to store the result

		  //here i stands for the subtable index in this example it equeals three
		  for(int i=0;i<NodeValueNumber;i++) {
			  subtable[i]=new Vector(probabilityTable.subList(i*splitrange,i*splitrange+splitrange));//tableprobabilitytable is the old table vectore
		  }//this initialize the three subtables

		  //here i stands for the subtable index also equeals to the index of normnumber
		  for(int i=0;i<NodeValueNumber;i++) {
			  double normalizationNumber=0;

			  for(int j=0;j<splitrange;j++) {
				  normalizationNumber = normalizationNumber + cast_to_double(subtable[i].get(j).toString());}
				  normNumber[i]=normalizationNumber;
			  }

			  if (splitTableIndex==-1) {
				  Vector priorTable = new Vector();
				  double allSumUp=0;

				  for(int i=0;i<NodeValueNumber;i++) {
					  allSumUp=allSumUp+normNumber[i];
				  }

				  for (int i=0;i<NodeValueNumber; i++) {
					  priorTable.addElement(String.valueOf(normNumber[i]/allSumUp));
				  }

				  return priorTable;
			  }

			  else {
				  int requestNodeValueNumber = nodesGroup.getNodeByName((String)(parentList.elementAt(splitTableIndex))).getArity();

				  //decided by how many next(younger) sibeling and how many values those sibling has
				  for(int i=splitTableIndex;i<parentList.size();i++) {
					  jumpnumber=jumpnumber*(nodesGroup.getNodeByName((String)(parentList.elementAt(i))).getArity());
				  }//this loop get the jump number include the request node itself i

				  //decided by how many next(younger) sibeling and how many values those sibling has
				  for(int i=splitTableIndex;i<parentList.size();i++) {
					  if (i+1==parentList.size()){}//if it is the last request node
					  else {
						  accumulationNumber=accumulationNumber*(nodesGroup.getNodeByName((String)(parentList.elementAt(i+1))).getArity());
					  }
				  }//this loop get the accumulationNumber exclude the request node


				  //the followinig 4 loops are together
				  //this decide do b1 or b2 or b3, this loop must be outer loop
				  for(int n=0;n< requestNodeValueNumber;n++) {
					  //this decide row number which decide d1 d2 row
				  for (int i=0;i<NodeValueNumber;i++) {
						  double probabilitysum = 0; //each time you need to initialize it come into loop

						  //when n is 0 it will decide to do b1
						  //and each time b1 will partially accumulated. then it jump to next start point to start next partial
						  //accumulation again untill it this loop will control jump
						  for(int p=n*accumulationNumber;p<splitrange;p=p+jumpnumber) {
							  //this loop control accumulation each time it starts at p
							  //actually p decide to do b1,b2 or b3
							  for(int j=p;j<p+accumulationNumber;j++) {
								  probabilitysum = probabilitysum+cast_to_double((String)(subtable[i].elementAt(j)));
							  }

						  }//the above two loop can finish the accumulation of b1 or b2 in one row
						  probabilitysum = probabilitysum/normNumber[i];
						  String stringOfprobabilitysum =""+probabilitysum;
						  putbackprobability.add(stringOfprobabilitysum);
					  }//this loop finish can finish one group of columns such as b1 group

				  }//after finish this last loop we are finished b2 and b3 group of columns

				  return putbackprobability;
			  }
		}

		 private double cast_to_double(String s) {
			Double doubleOb = new Double(s);
			return doubleOb.doubleValue();
		}

		public void printVerbose(){
			System.out.println("Node name: " + nodeName);
			System.out.println("Arity: " + getArity());
			System.out.print("States: ");

			for (int i = 0; i < possibleValueList.size(); i++){
				System.out.print((String) possibleValueList.elementAt(i) + " ");
			}

			System.out.print("\nParents: ");
			if (this.parentList.size()==0){System.out.print("Empty");}
			else{
				for (int i = 0; i < parentList.size(); i++){
					System.out.print(getParent(i).getName() + " ");
				}
			}
			System.out.println("");
			System.out.println("Probability table: ");
			for (int i = 0; i < probabilityTable.size(); i++){
				System.out.print(((Double) probabilityTable.elementAt(i)).doubleValue() + " ");
			}
			System.out.println("\n-----------------\n");
		}

	}


	

