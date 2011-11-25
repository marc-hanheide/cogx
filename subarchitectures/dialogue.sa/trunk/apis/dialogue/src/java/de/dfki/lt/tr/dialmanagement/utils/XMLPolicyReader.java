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
import java.util.LinkedList;
import java.util.List;

import org.xml.sax.InputSource;
import org.xml.sax.SAXException;


import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.AlternativeAction;
import de.dfki.lt.tr.dialmanagement.data.actions.DialStateAction;
import de.dfki.lt.tr.dialmanagement.data.actions.EventAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.actions.MotorAction;
import de.dfki.lt.tr.dialmanagement.data.actions.PhonstringAction;
import de.dfki.lt.tr.dialmanagement.data.actions.RemoveVariableAction;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.DialStateCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.EventCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.IntentionCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.PhonstringCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.TimeoutCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.EmptyCondition;
import org.apache.log4j.Logger;
import org.apache.xerces.parsers.DOMParser;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;


/**
 * Utility for constructing a new dialogue policy from a finite-state specification
 * encoded in a XML format
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 09/10/2010
 */

public class XMLPolicyReader {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	private static Logger logger = Logger.getLogger("xmlpolicyreader");

	// counter for forging edge identifiers
	private static int count = 0 ;

	
	

	// ==============================================================
	// POLICY CONSTRUCTION METHODS
	// ==============================================================

	
	
	/**
	 * Construct a new dialogue policy from a XML file specification
	 * 
	 * @param policyFile the path to the policy file
	 * @return the dialogue policy
	 * @throws DialogueException if a formatting error is found
	 */
	public static DialoguePolicy constructPolicy (String policyFile) throws DialogueException {

		DOMParser parser = new DOMParser();
		File f = new File(policyFile);
		DialoguePolicy policy = new DialoguePolicy();

		try {
			FileReader reader = new FileReader(f);
			InputSource source = new InputSource(reader);
			parser.parse(source);
			Document testDoc = parser.getDocument();
			String topCategory = testDoc.getChildNodes().item(0).getNodeName();

			if (topCategory.equals("policy")) {
				NodeList childNodes = testDoc.getChildNodes().item(0).getChildNodes();

				int nbProcessedSections = 0 ;
				for (int i = 0 ; i < childNodes.getLength() ; i++) {
					Node xmlNode = childNodes.item(i);

					if (xmlNode.getNodeName().equals("nodes")) {
						addNodesToPolicy (xmlNode, policy);
						nbProcessedSections++;
					}
					else if (xmlNode.getNodeName().equals("edges")) {
						addEdgesToPolicy (xmlNode, policy);
						nbProcessedSections++;
					}
					else if (xmlNode.getNodeName().equals("actions")) {
						addActionsToPolicy(xmlNode, policy);
						nbProcessedSections++;
					}
					else if (xmlNode.getNodeName().equals("conditions")) {
						addConditionsToPolicy(xmlNode, policy);
						nbProcessedSections++;
					}	
				}	
				if (nbProcessedSections < 4) {
					throw new DialogueException("wrongly formatted policy file");
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

		debug("Policy construction is successfull!");
		return policy;
	}




	// ==============================================================
	// NODE CREATION METHODS
	// ==============================================================


	
	
	/**
	 * Adding the nodes specified in the XML node to the dialogue policy
	 * 
	 * @param topXMLNode the XML node (within the <nodes</nodes> tag)
	 * @param policy the dialogue policy
	 * @throws DialogueException if the file is ill-formatted
	 */
	public static void addNodesToPolicy (Node topXMLNode, DialoguePolicy policy) throws DialogueException {

		for (int i = 0 ; i < topXMLNode.getChildNodes().getLength() ; i++) {
			Node xmlNode = topXMLNode.getChildNodes().item(i);

			if (xmlNode.getNodeName().equals("node") && 
					xmlNode.getAttributes().getNamedItem("id") != null) {

				PolicyNode pnode = new PolicyNode(xmlNode.getAttributes().getNamedItem("id").getNodeValue());
				debug("adding node: " + pnode.getId());
				policy.addNode(pnode);

				if (Boolean.parseBoolean(xmlNode.getAttributes().getNamedItem("isInitial").getNodeValue())) {
					debug("setting node " + pnode.getId() + " as initial");
					policy.setNodeAsInitial(pnode.getId(),true);
				}

				if (Boolean.parseBoolean(xmlNode.getAttributes().getNamedItem("isFinal").getNodeValue())) {
					debug("setting node " + pnode.getId() + " as final");
					policy.setNodeAsFinal(pnode.getId(),true);
				}

				if (xmlNode.getAttributes().getNamedItem("action") != null) {
					String[] actionsStr = xmlNode.getAttributes().getNamedItem("action").getNodeValue().split(",");
					for (int j = 0 ; j < actionsStr.length ; j++) {
						pnode.addEmptyAction(actionsStr[j].trim());
					}
				}
				else if (!policy.isInitNode(pnode.getId())){
					log("WARNING: no action specified for node id " + pnode.getId());
				}

			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}



	// ==============================================================
	// EDGE CREATION METHODS
	// ==============================================================


	
	/**
	 * Adding the edges specified in the XML node to the dialogue policy
	 * 
	 * @param topXMLNode the XML node (within the <edges></edges> tags)
	 * @param policy the dialogue policy
	 * @throws DialogueException if the policy file is ill-formatted
	 */
	public static void addEdgesToPolicy (Node topXMLNode, DialoguePolicy policy) throws DialogueException {

		for (int i = 0 ; i < topXMLNode.getChildNodes().getLength() ; i++) {
			Node xmlNode = topXMLNode.getChildNodes().item(i);

			if (xmlNode.getNodeName().equals("edge") && 
					xmlNode.getAttributes().getNamedItem("source") != null && 
					xmlNode.getAttributes().getNamedItem("target") != null ) {

				String id ;
				if (xmlNode.getAttributes().getNamedItem("id") != null) {
					id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
				}
				else {
					id = forgeNewId();
				}
				String source = xmlNode.getAttributes().getNamedItem("source").getNodeValue();
				String target = xmlNode.getAttributes().getNamedItem("target").getNodeValue();

				if (xmlNode.getAttributes().getNamedItem("deactivated") == null) {

					// normal case, where the policy contains both the source and the target
					if (policy.hasNode(source) && policy.hasNode(target)) {
						PolicyEdge pedge = createEdge(xmlNode, id, source, target);
						debug("adding edge: " + pedge.getId());
						policy.addEdge(pedge) ;
					}

					// special case, when the source is indicated as *
					else if (source.equals("*") && policy.hasNode(target)) {
						debug("adding an edge at each possible node");
						int counter = 1;
						for (PolicyNode node : policy.getAllNodes()) {
							PolicyEdge pedge = createEdge(xmlNode, id+counter, node.getId(), target); 
							debug("adding edge: " + pedge.getId());
							policy.addEdge(pedge) ;						
						}
					}

					else {
						log("WARNING: the nodes specified for edge " + 
								xmlNode.getAttributes().getNamedItem("id") + " are not specified anywhere");
					}
				}
				else {
					log("edge " + id + " temporarily deactivated");
				} 
			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}



	/**
	 * Create a new edge given an xml node describing the conditions, plus an id, source and target
	 * @param xmlNode
	 * @param id
	 * @param source
	 * @param target
	 * @return
	 */
	private static PolicyEdge createEdge (Node xmlNode, String id, String source, String target) {

		PolicyEdge pedge = new PolicyEdge (id, source, target); 

		if (xmlNode.getAttributes().getNamedItem("condition") != null) {
			String[] conditionsStr = xmlNode.getAttributes().getNamedItem("condition").getNodeValue().split(",");
			for (int j = 0 ; j < conditionsStr.length ; j++) {
				pedge.addCondition(new EmptyCondition (conditionsStr[j].trim()));
				debug("adding condition " + conditionsStr[j].trim() + " to edge " + pedge.getId());
			}
		}	

		if (xmlNode.getAttributes().getNamedItem("condition") == null && 
				xmlNode.getAttributes().getNamedItem("precond") == null) {
			log("WARNING: no condition specified for edge id " + pedge.getId());
		}
		return pedge;
	}


	

	// ==============================================================
	// ACTION CREATION METHODS
	// ==============================================================


	
	
	/**
	 * Add the policy actions specified in the XML node to the dialogue policy
	 * 
	 * @param topXMLNode the XML node
	 * @param policy the dialogue policy
	 * @throws DialogueException if the policy file is ill-formatted
	 */
	public static void addActionsToPolicy (Node topXMLNode, DialoguePolicy policy) throws DialogueException {

		for (int i = 0 ; i < topXMLNode.getChildNodes().getLength() ; i++) {
			Node xmlNode = topXMLNode.getChildNodes().item(i);

			if (xmlNode.getNodeName().equals("action") && 
					xmlNode.getAttributes().getNamedItem("id") != null && 
					xmlNode.getAttributes().getNamedItem("content") != null && 
					xmlNode.getAttributes().getNamedItem("type") != null) {	

				// create the action
				AbstractAction paction = createAction(xmlNode);

				// attach the action to its nodes
				attachActionToNodes(paction, policy);
				
			}	
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}


	/**
	 * Create the action as specified by the XML node
	 * 
	 * @param xmlNode the xml node providing the information
	 * @return the created action
	 * @throws DialogueException if the node is ill-formed
	 */
	private static AbstractAction createAction (Node xmlNode) throws DialogueException {

		String id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
		String content = xmlNode.getAttributes().getNamedItem("content").getNodeValue();

		AbstractAction paction;
		debug("creating action: " + id);

		if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("intention")) {
			debug("setting type of action as: intention");
			paction = new IntentionAction (id, FormulaUtils.constructFormula(content));
			if (xmlNode.getAttributes().getNamedItem("status") != null) {
				if (xmlNode.getAttributes().getNamedItem("status").getNodeValue().equals("communicative")) {
					((IntentionAction)paction).setStatus(IntentionAction.COMMUNICATIVE);
				}
				else if (xmlNode.getAttributes().getNamedItem("status").getNodeValue().equals("attributed")) {
					((IntentionAction)paction).setStatus(IntentionAction.ATTRIBUTED);
				}
				else if (xmlNode.getAttributes().getNamedItem("status").getNodeValue().equals("private")) {
					((IntentionAction)paction).setStatus(IntentionAction.PRIVATE);
				}
			}

		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("motoraction")) {
			debug("setting type of action as: motor action");
			paction = new MotorAction(id, content);
		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("phonstring")) {
			debug("setting type of action as: phonstring");
			paction = new PhonstringAction(id, content);
		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("dialstate")) {
			debug("setting type of action as: dialstate");
			paction = new DialStateAction(id, FormulaUtils.constructFormula(content));
		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("event")) {
			debug("setting type of action as: event");
			paction = new EventAction(id, FormulaUtils.constructFormula(content));
		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("rmvar")) {
			debug("setting type of action as: rmvar");
			paction = new RemoveVariableAction(id, content);
		}
		else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("alternativeaction")) {
			debug("setting type of action as: dialstate");
			// we assume here that the action are phonstrings!!
			String[] split = content.split(",");
			List<AbstractAction> subactions = new LinkedList<AbstractAction>();
			for (int i = 0; i < split.length ; i++) {
				subactions.add(new PhonstringAction(id+i, split[i].replace("[", "").replace("]", "")));
			}
			paction = new AlternativeAction(id, subactions);
		}
		else {
			throw new DialogueException("ERROR: not accepted type");
		}

		return paction;
	}
	
	
	
	/**
	 * Attach the given action to its nodes in the policy
	 * 
	 * @param action the action to attach
	 * @param policy the policy to modify
	 */
	private static void attachActionToNodes (AbstractAction action, DialoguePolicy policy) {

		// modifying the nodes pointing to the action to integrate it
		boolean foundNode = false;
		for (PolicyNode node : policy.getAllNodes()) {
			for (String actionId: node.getEmptyActions()) {
				if (actionId.equals(action.getId())) {
					debug("modifying node " + node.getId() + " to link to action " + action.getId());
					node.addAction(action);
					foundNode = true;
				}
			}

		}
		if (!foundNode) {
			log("WARNING: no node associated with action " + action.getId());
		}
	}
	
	

	// ==============================================================
	// CONDITION CREATION METHODS
	// ==============================================================


	

	/**
	 * Add the conditions specified in the XML node to the dialogue policy
	 * 
	 * @param topXMLNode the XML node
	 * @param policy the dialogue policy
	 * @throws DialogueException if the policy is ill-formatted
	 */
	public static void addConditionsToPolicy (Node topXMLNode, DialoguePolicy policy) throws DialogueException {

		for (int i = 0 ; i < topXMLNode.getChildNodes().getLength() ; i++) {
			Node xmlNode = topXMLNode.getChildNodes().item(i);

			if (xmlNode.getNodeName().equals("condition") && 
					xmlNode.getAttributes().getNamedItem("id") != null && 
					xmlNode.getAttributes().getNamedItem("content") != null &&
					xmlNode.getAttributes().getNamedItem("type") != null) {

				
				AbstractCondition pcond = createCondition (xmlNode);
				debug("created condition: " + pcond.getId());

				attachConditiontoEdges (pcond, policy);
			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}
	
	
	/**
	 * Create the policy condition given the information provided in the XML node
	 * 
	 * @param xmlNode the XML node
	 * @return the created condition
	 * @throws DialogueException if the node is ill-formed
	 */
	private static AbstractCondition createCondition(Node xmlNode) throws DialogueException {

		AbstractCondition pcond;
		
		String id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
		String content = xmlNode.getAttributes().getNamedItem("content").getNodeValue();

		
		String typeString = xmlNode.getAttributes().getNamedItem("type").getNodeValue();

		if (typeString.equals("intention")) {
			pcond = new IntentionCondition(id, FormulaUtils.constructFormula(content));
		}
		else if (typeString.equals("event")) {
			pcond = new EventCondition(id, FormulaUtils.constructFormula(content));
		}
		else if (typeString.equals("phonstring")) {
			pcond = new PhonstringCondition(id, content);
		}
		else if (typeString.equals("timeout")) {
			int timeout = Integer.parseInt(content);
			pcond = new TimeoutCondition (id, timeout);
		}
		else if (typeString.equals("dialstate")) {
			dFormula dialCond = FormulaUtils.constructFormula(content);
			if (dialCond instanceof ModalFormula) {
				pcond = new DialStateCondition(id, (ModalFormula)dialCond);
			}
			else {
				throw new DialogueException("Error, dialogue state condition not properly formatted");
			}
		}
		else {
			throw new DialogueException("Error, condition type not accepted: \"" + typeString + "\"");
		}

		if (xmlNode.getAttributes().getNamedItem("lowerProb") != null) {
			try {
				float lowerProb = Float.parseFloat(xmlNode.getAttributes().getNamedItem("lowerProb").getNodeValue());
				pcond.setMinimumProb(lowerProb);
			}
			catch (NumberFormatException e) {}
		}

		if (xmlNode.getAttributes().getNamedItem("higherProb") != null) {
			try {
				float higherProb = Float.parseFloat(xmlNode.getAttributes().getNamedItem("higherProb").getNodeValue());
				pcond.setMaximumProb(higherProb);
			}
			catch (NumberFormatException e) {}
		}
		
		return pcond;
	}

	
	
	/**
	 * Attach the condition to its edges
	 * 
	 * @param pcond the condition to attach
	 * @param policy the policy to modify
	 */
	private static void attachConditiontoEdges (AbstractCondition pcond, DialoguePolicy policy) {
		
		// modifying the edges pointing to the condition to integrate it
		boolean foundEdge = false;
		for (PolicyEdge edge : policy.getAllEdges()) { 

			List<AbstractCondition> toRemove = new LinkedList<AbstractCondition>();
			List<AbstractCondition> toAdd = new LinkedList<AbstractCondition>();
			for (AbstractCondition condition : edge.getConditions()) {

				if (condition != null && condition instanceof EmptyCondition 
						&& condition.getId().equals(pcond.getId())) {	
					debug("modifying edge " + edge.getId() + " to link to condition " + pcond.getId());
					toRemove.add(condition);
					toAdd.add(pcond);
					foundEdge = true;
				}
			}
			for (AbstractCondition tr : toRemove) { edge.removeCondition(tr); }
			for (AbstractCondition ta : toAdd) { edge.addCondition(ta);}
		}	
		if (!foundEdge) {
			log("WARNING: no edge associated with condition " + pcond.getId());
		}
	}

	
	

	// ==============================================================
	// UTILITY METHODS
	// ==============================================================


	
	
	/**
	 * Forge a new edge identifier
	 * @return
	 */
	private static String forgeNewId() {
		count++;
		return "edge" + count;
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (logger != null) {
			logger.info(s);
		}
		else if (LOGGING) {
			System.out.println("[xmlreader LOG] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (logger != null) {
			logger.debug(s);
		}
		else if (DEBUG) {
			System.out.println("[xmlreader DEBUG] " + s);
		}
	}
}
