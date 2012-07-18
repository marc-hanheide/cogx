// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package de.dfki.lt.tr.dialmanagement.utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.apache.xerces.parsers.DOMParser;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

//import com.sun.org.apache.xerces.internal.parsers.DOMParser;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.AlternativeAction;
import de.dfki.lt.tr.dialmanagement.data.actions.MotorAction;
import de.dfki.lt.tr.dialmanagement.data.actions.PhonstringAction;
import de.dfki.lt.tr.dialmanagement.data.conditions.PhonstringCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.TimeoutCondition;

/**
 * Utility class for extract dialogue policies specified in the TNO format
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/11/2010
 * 
 */
public class TNOPolicyReader {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the standard response waiting time
	public static final int STANDARD_BREAK = 2000;




	// ==============================================================
	// POLICY CONSTRUCTION METHODS
	// ==============================================================

	
	
	/**
	 * Construct a new dialogue policy based on the specification provided in the
	 * policy file (assuming the policy is encoded in the TNO format)
	 * 
	 * @param policyFile the pathname of the policy
	 * @return the created policy
	 * 
	 * @throws DialogueException
	 */
	public static DialoguePolicy constructPolicy (String policyFile) throws DialogueException {

		DOMParser parser = new DOMParser();
		File f = new File(policyFile);
		DialoguePolicy policy = new DialoguePolicy();

		try {
			FileReader reader = new FileReader(f);
			InputSource source = new InputSource(reader);
			parser.parse(source);
			Document fullDoc = parser.getDocument();
			fullDoc.getDocumentElement().normalize();

 
			if (fullDoc.getFirstChild().getNodeName().equals("dialog")) {
				for (int i = 0 ; i < fullDoc.getFirstChild().getChildNodes().getLength(); i++) {
					Node curNode = fullDoc.getFirstChild().getChildNodes().item(i);
					if (curNode.getNodeName().equals("subdialogs")) {

						debug("looping on the subdialogs...");

						for (int j = 0 ; j < curNode.getChildNodes().getLength(); j++)  {
							Node subdialogNode = curNode.getChildNodes().item(j);
							if (subdialogNode.getNodeName().equals("subdialog")) {

								// extracting the subdialog policy
								DialoguePolicy subPolicy = extractPolicyFromSubdialog (subdialogNode);

								// adding a break between the policies
								policy = concatenatePolicies (policy, subPolicy, createTimeBreakCondition());

								debug("size of policy: (" + policy.getAllNodes().size() + ", " + policy.getAllEdges().size() + ")");
							}
						}
					}
				}
			}


		} catch (FileNotFoundException e) {
			throw new DialogueException ("policy file not found: " + e.getMessage());
		} 
		catch (IOException e) {
			throw new DialogueException ("I/O error, wrongly formatted policy file: " + e.getMessage());
		}
		catch (SAXException e) {
			throw new DialogueException ("wrongly formatted policy file: " + e.getMessage());
		}

		//	log(policy.toString());	

		return policy;
	}



	// ==============================================================
	// SUBPOLICY CREATION METHODS
	// ==============================================================

	
	
	/**
	 * Extract the dialogue policy from a subdialog specified in the XML node "subdialog",
	 * in the TNO format
	 * 
	 * @param subdialog the XML node 
	 * @return the created policy
	 * 
	 * @throws DialogueException
	 */
	private static DialoguePolicy extractPolicyFromSubdialog (Node subdialog) throws DialogueException {

		DialoguePolicy newPolicy = new DialoguePolicy();

		debug("constructing the policy from subdialog: " + subdialog.getAttributes().getNamedItem("id").getNodeValue());

		for (int i = 0 ; i < subdialog.getChildNodes().getLength(); i++) {
			Node subsection = subdialog.getChildNodes().item(i);

			debug("current subsection: " + subsection.getNodeName());

			if (subsection.getNodeName().equals("remark")) {
				DialoguePolicy subpolicy = extractPolicyFromRemark (subsection);
				newPolicy = concatenatePolicies (newPolicy, subpolicy, createTimeBreakCondition());
			}
			else if (subsection.getNodeName().equals("question")) {
				DialoguePolicy subpolicy = extractPolicyFromQuestion(subsection);
				newPolicy = concatenatePolicies (newPolicy, subpolicy, createTimeBreakCondition());
			}

			else if (subsection.getNodeName().equals("environmental_action")) {
				DialoguePolicy subpolicy = extractPolicyFromEnvironmentalAction(subsection);
				newPolicy = concatenatePolicies (newPolicy, subpolicy, createTimeBreakCondition());
			}

			else if (!subsection.getNodeName().equals("#text")){
				//			throw new DialogueException ("ERROR, unhandled dialogue type: " + subsection.getNodeName());
			}
		}

		return newPolicy;
	}



	/**
	 * Extract the dialogu policy from a "question" XML node in the TNO format
	 * 
	 * @param question the XML node
	 * @return the created policy
	 * @throws DialogueException
	 */
	private static DialoguePolicy extractPolicyFromQuestion(Node question) throws DialogueException {

		debug("constructing the policy from question node...");
		DialoguePolicy newPolicy = new DialoguePolicy();

		if (question.getAttributes().getNamedItem("id") != null && 
				question.getAttributes().getNamedItem("text") != null) {

			String questionId = question.getAttributes().getNamedItem("id").getNodeValue();
			String questionText = question.getAttributes().getNamedItem("text").getNodeValue();

			PolicyNode questionNode = new PolicyNode(questionId, Arrays.asList(createAbstractAction(questionId+"A", questionText)));
			newPolicy.addNode(questionNode);
			newPolicy.setNodeAsInitial(questionNode.getId(),true);

			for (int j = 0 ; j < question.getChildNodes().getLength(); j++) {
				Node curNode = question.getChildNodes().item(j);

				if (curNode.getNodeName().equals("answers")) {
					for (int i = 0 ; i < curNode.getChildNodes().getLength(); i++) {
						Node subnode = curNode.getChildNodes().item(i);

						if (subnode.getNodeName().equals("answer") && 
								subnode.getAttributes().getNamedItem("id") != null && 
								subnode.getAttributes().getNamedItem("text") != null) {

							String answerText = subnode.getAttributes().getNamedItem("text").getNodeValue();

							for (int k = 0 ; k < subnode.getChildNodes().getLength(); k++) {

								Node subsubnode = subnode.getChildNodes().item(k);

								if (subsubnode.getNodeName().equals("remark") && 
										subsubnode.getAttributes().getNamedItem("id") != null && 
										subsubnode.getAttributes().getNamedItem("text") != null) {

									String responseId = subsubnode.getAttributes().getNamedItem("id").getNodeValue();

									if (newPolicy.hasNode(responseId)) {
										responseId = responseId + "bis";
									}
									String responseText = subsubnode.getAttributes().getNamedItem("text").getNodeValue();

									PolicyNode responseNode = new PolicyNode(responseId, 
											Arrays.asList(createAbstractAction(responseId+"A", responseText)));
									newPolicy.addNode(responseNode);

									for (PolicyEdge answerEdge : createAnswerEdges(answerText,questionNode,responseNode)) {
										newPolicy.addEdge(answerEdge);
									}

									if (subsubnode.getAttributes().getNamedItem("loopsuccessor") != null &&
											!subsubnode.getAttributes().getNamedItem("loopsuccessor").getNodeValue().equals("")	) {

										String targetNodeId = subsubnode.getAttributes().
										getNamedItem("loopsuccessor").getNodeValue();
										PolicyEdge edge = new PolicyEdge("timebreak-"  + new Random().nextInt(), 
												responseNode.getId(), targetNodeId, createTimeBreakCondition());
										newPolicy.addEdge(edge);
										debug("added a new edge " + edge.getId() + " between " 
												+ responseNode.getId() + " and " + targetNodeId);

									}
									else {
										newPolicy.setNodeAsFinal(responseNode.getId(),true);
									}
								}

							}

						}
					}
				}
			}

		}

		debug("constructed subpolicy from question node: " + newPolicy.toString());

		return newPolicy;
	}




	/**
	 * Extract a dialogue policy from an "environmental action" XML node specified in the
	 * TNO format
	 * 
	 * @param eaction the XML node
	 * @return the created policy
	 * @throws DialogueException
	 */
	private static DialoguePolicy extractPolicyFromEnvironmentalAction (Node eaction) throws DialogueException {

		debug("constructing the policy from environmental action node...");

		DialoguePolicy newPolicy = new DialoguePolicy();

		if (eaction.getAttributes().getNamedItem("id") != null && 
				eaction.getAttributes().getNamedItem("action") != null) {

			String id = eaction.getAttributes().getNamedItem("id").getNodeValue();
			String text = eaction.getAttributes().getNamedItem("action").getNodeValue();

			PolicyNode newNode = new PolicyNode(id, Arrays.asList((AbstractAction)new MotorAction(id+"A", text)));
	//		for (PhonstringAction action : newNode.getActions()) {
	//			action.setType(AbstractAction.PHONSTRING);
	//		}
			newPolicy.addNode(newNode);
			newPolicy.setNodeAsInitial(newNode.getId(),true);
			newPolicy.setNodeAsFinal(newNode.getId(),true);
		} 
		else {
			throw new DialogueException("ERROR, wrong formatting in eaction");
		}

		//	debug("constructed subpolicy from remark node: " + newPolicy.toString());

		return newPolicy;
	}


	
	/**
	 * Extract a dialogue policy specified in a "remark" XML node specified in the TNO format
	 * 
	 * @param remark the XML node
	 * @return the created policy
	 * @throws DialogueException
	 */
	private static DialoguePolicy extractPolicyFromRemark (Node remark) throws DialogueException {
		
		DialoguePolicy policy = new DialoguePolicy();
		
		debug("constructing the policy from remark node...");

		if (remark.getAttributes().getNamedItem("id") != null && 
				remark.getAttributes().getNamedItem("text") != null) {

			String id = remark.getAttributes().getNamedItem("id").getNodeValue();
			
			String text = remark.getAttributes().getNamedItem("text").getNodeValue();

			PolicyNode newNode = new PolicyNode(id, Arrays.asList((AbstractAction)new PhonstringAction(id+"A", text)));
	//		for (AbstractAction action : newNode.getActions()) {
	//			action.setType(AbstractAction.PHONSTRING);
	//		}
			policy.addNode(newNode);
			policy.setNodeAsInitial(newNode.getId(),true);

			if (remark.getAttributes().getNamedItem("loopsuccessor") != null &&
					!remark.getAttributes().getNamedItem("loopsuccessor").getNodeValue().equals("")	) {

				String targetNodeId = remark.getAttributes().getNamedItem("loopsuccessor").getNodeValue();
				PolicyEdge edge = new PolicyEdge("timebreak-"  + new Random().nextInt(), 
						newNode.getId(), targetNodeId, createTimeBreakCondition());
				policy.addEdge(edge);
				debug("in remark, adding a new edge " + edge.getId() + " targetting " + targetNodeId);
			}
			else {
				policy.setNodeAsFinal(newNode.getId(),true);
			} 
		} 
		else {
			throw new DialogueException("ERROR, wrong formatting in remark");
		}

		return policy;
	}

	
	// ==============================================================
	// ACTION AND EDGE CREATION METHODS
	// ==============================================================

	
	/**
	 * Create a new policy action based on the specified response
	 *  
	 * @param responseId the identifier 
	 * @param responseText the text for the policy action
	 * @return the created policy action
	 */
	private static AbstractAction createAbstractAction (String responseId, String responseText) {

		AbstractAction action ;
		if (!responseText.contains("#")) {
			action = new PhonstringAction(responseId+"A", responseText);
		}
		else {
			String[] split = responseText.split("#");
			List<AbstractAction> alternativeActions = new LinkedList<AbstractAction>();
			for (int i = 0 ; i < split.length ; i++) {
				alternativeActions.add(new PhonstringAction(responseId+"A"+i, split[i]));
			}
			action =  new AlternativeAction(responseId+"A", alternativeActions);
		}
		return action;
	}



	/**
	 * Create a collection of answer edges between the source and answer node, using the 
	 * alternative answers provided in the answerText
	 *  
	 * @param answerText a set of alternative answers separated by the # character
	 * @param initNode the init node
	 * @param targetNode the target node
	 * @return the collection of edges
	 */
	private static Collection<PolicyEdge> createAnswerEdges 
	(String answerText, PolicyNode initNode, PolicyNode targetNode) {

		String[] alternativeAnswers = answerText.split("#");
		List<PolicyEdge> edges = new LinkedList<PolicyEdge>();
		for (int i = 0 ; i < alternativeAnswers.length ; i++) {
			dFormula condition ;
			if (!alternativeAnswers[i].equals("")) {
				condition = new ElementaryFormula(0, alternativeAnswers[i]);
			}
			else {
				condition = new UnderspecifiedFormula(0, "");
			}
			PhonstringCondition answerCond = new PhonstringCondition(alternativeAnswers[i] + "C", alternativeAnswers[i]);
			PolicyEdge answerEdge = new PolicyEdge(alternativeAnswers[i] + new Random().nextInt(), 
					initNode.getId(), targetNode.getId(), answerCond);	
			edges.add(answerEdge);
		}

		return edges;

	}




	// ==============================================================
	// UTILITY METHODS
	// ==============================================================


	
	/**
	 * Create a new policy condition encoding a time break of a specified time
	 * 
	 * @return the created condition
	 */
	private static TimeoutCondition createTimeBreakCondition () {
		String id = "timebreak-" + new Random().nextInt() ;
		TimeoutCondition cond = 
			new TimeoutCondition(id+ "C", STANDARD_BREAK);
		return cond;
	}


	/**
	 * Concatenate the two dialogue policies, adding bridging edges between the
	 * final nodes of the first policy and the initial node of the second
	 * 
	 * @param policy the first policy
	 * @param subPolicy the second policy
	 * @param bridgeCondition the condition for the edges bridging the two policies
	 * @return a new policy which is the concatenation of the two
	 * @throws DialogueException
	 */
	private static DialoguePolicy concatenatePolicies (
			DialoguePolicy policy, DialoguePolicy subPolicy, 
			TimeoutCondition bridgeCondition) throws DialogueException {

		//	debug("initial policy before conc: " + policy.toString());
		debug("nb edges of subpolicy: " + subPolicy.getAllEdges().size());
		if (policy.getAllNodes().size() == 0) {
			return subPolicy;
		}
		else {
			policy.concatenatePolicy(subPolicy, bridgeCondition);
			//		debug("policy after conc: " + newPolicy);

			return policy;

		}
	}
	
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[tnopolicyreader] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[tnopolicyreader] " + s);
		}
	}
}
