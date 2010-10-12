// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@dfki.de)                                                                
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

import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import com.sun.org.apache.xerces.internal.parsers.DOMParser;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyCondition;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;


/**
 * Utility for constructing a new dialogue policy from a finite-state specification
 * encoded in a XML format
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 09/10/2010
 */

public class XMLPolicyReader {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
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
			throw new DialogueException ("wrongly formatted policy file: " + e.getMessage());
		} 
		catch (IOException e) {
			throw new DialogueException ("wrongly formatted policy file: " + e.getMessage());
		}
		catch (SAXException e) {
			throw new DialogueException ("wrongly formatted policy file: " + e.getMessage());
		}

		return policy;
	}

	
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
					
				if (Boolean.parseBoolean(xmlNode.getAttributes().getNamedItem("isInitial").getNodeValue())) {
					debug("setting node " + pnode.getId() + " as initial");
					policy.setNodeAsInitial(pnode);
				}
				
				if (Boolean.parseBoolean(xmlNode.getAttributes().getNamedItem("isFinal").getNodeValue())) {
					debug("setting node " + pnode.getId() + " as final");
					policy.setNodeAsFinal(pnode);
				}
				
				debug("we are here...");
				if (xmlNode.getAttributes().getNamedItem("action") != null) {
					debug("and here...");
					pnode.setPolicyAction(new PolicyAction(xmlNode.getAttributes().getNamedItem("action").getNodeValue()));
				}
				else if (!pnode.isInitialNode()){
					log("WARNING: no action specified for node id " + pnode.getId());
				}
				
				debug("adding node: " + pnode.getId());
				policy.addNode(pnode);
			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}
	

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
					xmlNode.getAttributes().getNamedItem("id") != null && 
					xmlNode.getAttributes().getNamedItem("source") != null && 
					xmlNode.getAttributes().getNamedItem("target") != null ) {
				
				String id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
				String source = xmlNode.getAttributes().getNamedItem("source").getNodeValue();
				String target = xmlNode.getAttributes().getNamedItem("target").getNodeValue();
				
				if (xmlNode.getAttributes().getNamedItem("deactivated") == null) {
				
					if (source.equals("*") && policy.hasNode(target)) {
						debug("adding an edge at each possible node");
						
						int counter = 1;
						for (PolicyNode node : policy.getNodes()) {
							
							PolicyEdge pedge = new PolicyEdge (id+counter, node, policy.getNode(target)); 
							counter++;
							if (xmlNode.getAttributes().getNamedItem("condition") != null) {
								pedge.setCondition(new PolicyCondition (xmlNode.getAttributes().getNamedItem("condition").getNodeValue()));
							}		
							else {
								log("WARNING: no condition specified for edge id " + pedge.getId());
							}
							debug("adding edge: " + pedge.getId());
							policy.addEdge(pedge, node, policy.getNode(target)) ;						
						}
					}
					
					
					else if (policy.hasNode(source) && policy.hasNode(target)) {	
						PolicyEdge pedge = new PolicyEdge (id, policy.getNode(source), policy.getNode(target)); 
						
						if (xmlNode.getAttributes().getNamedItem("condition") != null) {
							pedge.setCondition(new PolicyCondition (xmlNode.getAttributes().getNamedItem("condition").getNodeValue()));
						}		
						else {
							log("WARNING: no condition specified for edge id " + pedge.getId());
						}
						debug("adding edge: " + pedge.getId());
						policy.addEdge(pedge, policy.getNode(source), policy.getNode(target)) ;
				}
				else {
					log("WARNING: the nodes specified for edge " + xmlNode.getAttributes().getNamedItem("id") + " are not specified anywhere");
				}
				}
				else {
					debug("edge " + id + " temporarily deactivated");
				} 
			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}
	
	
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
					xmlNode.getAttributes().getNamedItem("content") != null) {	

				String id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
				String content = xmlNode.getAttributes().getNamedItem("content").getNodeValue();
				
				PolicyAction paction = new PolicyAction(id, content);
				debug("creating action: " + paction.getId());
				
				if (xmlNode.getAttributes().getNamedItem("type") != null) {
					if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("communicativeintention")) {
						paction.setType(PolicyAction.COMMUNICATIVE_INTENTION);
					}
					else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("privateintention")) {
						debug("setting type of action as: private intention");
						paction.setType(PolicyAction.PRIVATE_INTENTION);
					}
					else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("attributedintention")) {
						debug("setting type of action as: attributed intention");
						paction.setType(PolicyAction.ATTRIBUTED_INTENTION);
					}
				}
				
				// modifying the nodes pointing to the action to integrate it
				boolean foundNode = false;
				for (PolicyNode node : policy.getNodes()) {
					 if (node.getPolicyAction().getId().equals(id)) {
						 debug("modifying node " + node.getId() + " to link to action " + paction.getId());
						 node.setPolicyAction(paction);
						 foundNode = true;
					 }
				}
				if (!foundNode) {
					log("WARNING: no node associated with action " + paction.getId());
				}
			}	
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}


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
					xmlNode.getAttributes().getNamedItem("content") != null) {
				
				String id = xmlNode.getAttributes().getNamedItem("id").getNodeValue(); 
				String content = xmlNode.getAttributes().getNamedItem("content").getNodeValue();
				
				PolicyCondition pcond = new PolicyCondition(id, content);
				
				// setting the type
				if (xmlNode.getAttributes().getNamedItem("type") != null) {
					if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("communicativeintention")) {
						pcond.setType(PolicyCondition.COMMUNICATIVE_INTENTION);
					}
					else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("intention")) {
						pcond.setType(PolicyCondition.INTENTION);
					}
					else if (xmlNode.getAttributes().getNamedItem("type").getNodeValue().equals("event")) {
						pcond.setType(PolicyCondition.EVENT);
					}
				}
				debug("creating condition: " + pcond.getId());

				// modifying the edges pointing to the condition to integrate it
				boolean foundEdge = false;
				for (PolicyEdge edge : policy.getEdges()) {
					if (edge.getCondition() != null && edge.getCondition().getId().equals(id)) {
						debug("modifying edge " + edge.getId() + " to link to condition " + pcond.getId());
						edge.setCondition(pcond);	
						foundEdge = true;
					}
				}	
				if (!foundEdge) {
					log("WARNING: no edge associated with condition " + pcond.getId());
				}
			}
			else if (!xmlNode.getNodeName().equals("#text") && !xmlNode.getNodeName().equals("#comment")){
				throw new DialogueException("wrongly formatted policy file for tag: " + xmlNode.getNodeName());
			}
		}
	}



	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[xmlreader] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[xmlreader] " + s);
		}
	}
}
