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
 * Created on Aug 5, 2004
 */
package edu.ksu.cis.bnj.ver3.inference.exact;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.Field;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.util.driver.Options;

/**
 * This class helps with Pearl's inference algorithm
 * for polytrees.
 * 
 * @author Julie Thornton
 */
public class PropNode {
    private BeliefNode node;
    private BeliefNetwork network;
    private PropNode[] parents;
    private PropNode[] children;
    
    private CPF marginals;
    private CPF pi;
    private CPF lambda;
    private CPF[] piMessages;
    private CPF[] lambdaMessages;
    
    private boolean isEvidence = false;
    private int evIndex = -1;
    
    public PropNode(BeliefNode thisNode, BeliefNetwork bnNetwork) {
        node = thisNode;
        network = bnNetwork;
    }
    
    public BeliefNode getNode() {
        return node;
    }
    
    //add lambda message to array
    public void setLambdaMessage(int child, CPF message) {
       lambdaMessages[child] = message; 
       if (!isEvidence) {
           updateBelief();
           sendLambdaMessages(-1);
           sendPiMessages(child);
       }
    }
    
    //add pi message to array
    public void setPiMessage(int parent, CPF message) {
        piMessages[parent] = message;
        if (!isEvidence) {
            updateBelief();
            sendPiMessages(-1);
            boolean notUnit = false;
            for (int i = 0; i < lambda.size(); i++) {
                if (((ValueDouble)lambda.get(i)).getValue() != 1.0) {
                    notUnit = true;
                    break;
                }
            }
            if (notUnit) {
                sendLambdaMessages(parent);
            }
        }
    }
    
    public void propagateEvidence() {
        updateBelief();
        sendLambdaMessages(-1);
        sendPiMessages(-1);
    }
    
    //calculate a lambda message for each parent
    //call the "set" method for each parent PropNode
    public void sendLambdaMessages(int excludedIndex) {
        CPF message = null;
        int parentArity = 0;
        Value sum = new ValueDouble(0.0);
        Value product = new ValueDouble(1.0);
        Value lambdaVal = null;
        Value curEntry = null;
        CPF nodeCPF = node.getCPF();
        int logical[] = null;
        
        for (int i = 0; i < parents.length; i++) {
            parentArity = parents[i].getNode().getDomain().getOrder();
            message = new CPF(new BeliefNode[]{parents[i].getNode()});
            for (int j = 0; j < parentArity; j++) {
                for (int k = 0; k < lambda.size(); k++) {
                    lambdaVal = lambda.get(k);
                    for (int cpfIndex = 0; cpfIndex < nodeCPF.size(); cpfIndex++) {
                        curEntry = nodeCPF.get(cpfIndex);
                        logical = nodeCPF.realaddr2addr(cpfIndex);
                        
                        //check to see that current node value is in entry
                        //and to see that current parent value is in entry
                        if (logical[0] == k && logical[i+1] == j) {
                            
                            //multiply entry by corr. pi message values of other parents
                            for (int piIndex = 0; piIndex < piMessages.length; piIndex++) {
                                //check to make sure index is not current parent
                                if (piIndex != i) {
                                    product = Field.mult(product, piMessages[piIndex].get(logical[piIndex+1]));
                                }
                            }
                            //now multiply pi-product by current cpf entry
                            product = Field.mult(product, curEntry);
                            //now add product to sum and set product back to 1.0
                            sum = Field.add(sum, product);
                            product = new ValueDouble(1.0);
                        } 
                    }
                    //multiply inner sum by current lambda value
                    sum = Field.mult(sum, lambdaVal);
                    
                    //add sum to current message entry and set sum back to 0.0
                    message.put(j, Field.add(sum, message.get(j)));
                    sum = new ValueDouble(0.0);
                }
            }
            if (i != excludedIndex) {
                parents[i].setLambdaMessage(getParentIndex(i), message);
            }
        }
    }
    
    private void print(CPF message) {
        System.out.println(Options.getString(message));
    }
    
    private int getParentIndex(int index) {
        BeliefNode parent = parents[index].getNode();
        BeliefNode[] parentsChild = network.getChildren(parent);
        
        for (int i = 0; i < parentsChild.length; i++) {
            if (parentsChild[i].getName().equals(node.getName())) {
                return i;
            }
        }
        
        return -1;
    }
    
    //calculate a pi message for each child
    //call the "set" method for each child PropNode
    public void sendPiMessages(int excludedIndex) {
        //in case not done recently
        updateBelief();
        CPF message = new CPF(new BeliefNode[]{node});
        for (int i = 0; i < children.length; i++) {
            for (int j = 0; j < marginals.size(); j++) {
                if (isEvidence) {
                    if (evIndex == j) {
                        message.put(j, new ValueDouble(1.0));
                    }
                    else {
                        message.put(j, new ValueDouble(0.0));
                    }
                }
                else {
                    Value lambdaVal = lambdaMessages[i].get(j);
                    message.put(j, Field.divide(marginals.get(j), lambdaVal));
                }
            }
            if (i != excludedIndex) {
                children[i].setPiMessage(getChildIndex(i), message);
            }
            message = new CPF(new BeliefNode[]{node});
        }
    }
    
    private int getChildIndex(int index) {
        BeliefNode child = children[index].getNode();
        BeliefNode[] childParents = network.getParents(child);
        
        for (int i = 0; i < childParents.length; i++) {
            if (childParents[i].getName().equals(node.getName())) {
                return i;
            }
        }
        
        return -1;
    }
    
    //calculate current pi, lambda, and marginal values
    //given the current pi and lambda messages
    public void updateBelief() {
        updatePi();
        updateLambda();
        updateMarginals();
    }
    
    private void updatePi() {
        //if evidence, set pi equal to vector of all zeros
        //and one one, corresponding to the evidence index
        
        clearPi();
        
        if (isEvidence) {
            for (int i = 0; i < pi.size(); i++) {
                if (evIndex == i) {
                    pi.put(i, new ValueDouble(1.0));
                }
                else {
                    pi.put(i, new ValueDouble(0.0));
                }
            }
            return;
        }
        
        CPF probs = node.getCPF();
        CPF parentCPF = null;
		Value product = new ValueDouble(0.0);
		Value entry   = null;
		int[] logical = null;
        for (int i = 0; i < probs.size(); i++) {
			logical = probs.realaddr2addr(i);
			entry = probs.get(i);
			product = Field.add(product, entry);
			
			// stores entry*(all parent PI values) in product
			for (int j = 1; j < logical.length; j++) {
				parentCPF = piMessages[j-1];
				product = Field.mult(product, parentCPF.get(logical[j]));
			}
			pi.put(logical[0], Field.add(pi.get(logical[0]), product));
			product = new ValueDouble(0.0);
		}
    }
    
    private void clearPi() {
        for (int i = 0; i < pi.size(); i++) {
            pi.put(i, new ValueDouble(0.0));
        }
    }
    
    private void updateLambda() {
        Value product = new ValueDouble(1.0);
        for (int i = 0; i < lambda.size(); i++) {
            if (isEvidence) {
                if (evIndex == i) {
                    lambda.put(i, new ValueDouble(1.0));
                }
                else {
                    lambda.put(i, new ValueDouble(0.0));
                }
            }
            else {
                for (int j = 0; j < lambdaMessages.length; j++) {
                    product = Field.mult(product, lambdaMessages[j].get(i));
                }
                lambda.put(i, product);
                product = new ValueDouble(1.0);
            }
        }
    }
    
    private void updateMarginals() {
        //multiply corresponding pi and lambda values
        //and put them in the marginals CPF    
        Value product = null;
        Value sum = new ValueDouble(0.0);
        
        for (int i = 0; i < pi.size(); i++) {
            product = Field.mult(lambda.get(i), pi.get(i));
            marginals.put(i, product);
            sum = Field.add(sum, product);
        }
        
        //now normalize the marginals
        for (int i = 0; i < marginals.size(); i++) {
            product = marginals.get(i);
            marginals.put(i, Field.divide(product, sum));
        }
    }
    
    //return the computed marginals
    public CPF getMarginals() {
        //just in case we're not current...
        updateBelief();
        
        return marginals;
    }
    
    //set up the array of parents and children
    //set lambda values, lambda messages, and 
    //pi messages to 1.
    //Determine if the node is evidence.
    //Initialize the pi and marginal arrays
    public void initialize(PropNode[] allNodes) {
        //set up array of parents and children
        BeliefNode[] bnParents = network.getParents(node);
        BeliefNode[] bnChildren = network.getChildren(node);
        
        parents = new PropNode[bnParents.length];
        children = new PropNode[bnChildren.length];
        
        for (int i = 0; i < bnParents.length; i++) {
            parents[i] = getPropNode(bnParents[i], allNodes);
        }
        
        for (int i = 0; i < bnChildren.length; i++) {
            children[i] = getPropNode(bnChildren[i], allNodes);
        }
        
        //intialize lambda values, pi values, marginals
        lambda = new CPF(new BeliefNode[]{node});
        pi = new CPF(new BeliefNode[]{node});
        marginals = new CPF(new BeliefNode[]{node});
         
        for (int i = 0; i < lambda.size(); i++) {
            lambda.put(i, new ValueDouble(1.0));
        }
        
        initializeMessages();
        initializeEvidence();
    }
    
    private void initializeMessages() {
        //create message arrays
        piMessages = new CPF[parents.length];
        lambdaMessages = new CPF[children.length];
        
        //now fill each message with 1's
        for (int i = 0; i < parents.length; i++) {
            piMessages[i] = new CPF(new BeliefNode[]{parents[i].getNode()});
            for (int j = 0; j < piMessages[i].size(); j++) {
                piMessages[i].put(j, new ValueDouble(1.0));
            }
        }
        
        for (int i = 0; i < lambdaMessages.length; i++) {
            lambdaMessages[i] = new CPF(new BeliefNode[]{node});
            for (int j = 0; j < lambdaMessages[i].size(); j++) {
                lambdaMessages[i].put(j, new ValueDouble(1.0));
            }
        }
    }
    
    private void initializeEvidence() {
        isEvidence = node.hasEvidence();
        
        if (isEvidence) {
            evIndex = ((DiscreteEvidence) node.getEvidence()).getDirectValue();
        }
    }
    
    private PropNode getPropNode(BeliefNode curNode, PropNode[] nodeArray) {
        for (int i = 0; i < nodeArray.length; i++) {
            if (nodeArray[i].getNode().getName().equals(curNode.getName())) {
                return nodeArray[i];
            }
        }
        
        return null;
    }
}
