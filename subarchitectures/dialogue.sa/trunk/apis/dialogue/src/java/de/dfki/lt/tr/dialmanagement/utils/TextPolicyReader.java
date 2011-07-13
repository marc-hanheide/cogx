
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

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;


import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.actions.PhonstringAction;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.EventCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.IntentionCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.PhonstringCondition;


/**
 * Utility for constructing a new dialogue policy from a finite-state specification
 * in the text-based AT&T FSM format, associated with a file describing the actions and a
 * file describing the conditions on observations.
 * 
 * NOTE: the present format does not support many functionalities of the dialogue manager,
 * and is mainly maintained for backward compatibility
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 21/12/2010
 */

public class TextPolicyReader {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	

	// ==============================================================
	// POLICY CONSTRUCTION METHODS
	// ==============================================================

	

	/**
	 * Construct a new dialogue policy according to the specifications in the 3 configuration
	 * files (one for the policy, one for the conditions, one for the actions)
	 * 
	 * @param policyFile path of the policy file
	 * @param condFile path of the conditions file
	 * @param actionsFile path of the actions file
	 * @return the constructed dialogue policy
	 * @throws DialogueException if the files don't exist or are not correctly formatted
	 */
	public static DialoguePolicy constructPolicy(String policyFile, String condFile, 
			String actionsFile) throws DialogueException {

		try {
			// extracting the conditions
			HashMap<String,AbstractCondition> conditions = extractConditions (FileUtils.readfile(condFile));

			// extracting the actions
			HashMap<String,AbstractAction> actions = extractActions (FileUtils.readfile(actionsFile));

			// constructing the policy
			String policyText = FileUtils.readfile(policyFile);
			
			return constructPolicy(policyText, conditions, actions);

		} catch (IOException e) {
			String m = "ERROR: problem reading the files: {" + policyFile +  ", " + condFile +
			", " + actionsFile +  "}, abording the construction of the finite-state machine";
			throw new DialogueException(m);
		}	
	}



	/**
	 * Construct a new dialogue policy from a policy specification, a set of conditions and a set of actions
	 * @param text the policy text
	 * @param conditions the conditions
	 * @param actions the actions
	 * @return
	 * @throws DialogueException if the policy text is not well formatted
	 */
	public static DialoguePolicy constructPolicy (String text, HashMap<String, AbstractCondition> conditions, 
			HashMap<String, AbstractAction> actions) throws DialogueException {

		String[] lines = text.split("\n");

		DialoguePolicy policy = new DialoguePolicy();

		for (int i = 0 ; i < lines.length ; i++) {
			String curLine = lines[i];
			debug("parsing line: " + curLine);
			
			// line formatted as "inNode outNode condition"
			if ((curLine.split("\t").length == 3) || (curLine.split(" ").length == 3)) {
				addEdgeAndNodesToPolicy (curLine, policy, conditions, actions);
				
				// if it is the first line, set the first node as initial
				if (i == 0) {
					debug("init node: " + getStringInTabbedLine(curLine,0));
					policy.setNodeAsInitial(policy.getNode(getStringInTabbedLine(curLine, 0)).getId(),true);
				}
			}
			
			// line formatted as "finalNode"
			else if (curLine.split("\t").length == 1) {
				addFinalNodesToPolicy(curLine, policy);
			}
			else {
				throw new DialogueException("ERROR: line " + i + " in policy file is not correctly formatted");
			}
		}
		return policy;
	}


	/**
	 * Returns a list of conditions extracted from the specification text
	 * 
	 * @param condText the text specifying the conditions
	 * @return a hashmap containing the edges, indexed by their identifier
	 * @throws DialogueException if the specification text is ill-formated
	 */
	public static HashMap<String,AbstractCondition> extractConditions (String condText) throws DialogueException {

		String[] lines = condText.split("\n");

		HashMap<String,AbstractCondition> conditions = new HashMap<String, AbstractCondition>();

		for (int i = 0 ; i < lines.length ; i++) {
			String line = lines[i];

			String[] tabs = line.split("=");

			if (tabs.length == 2) {
				String condSymbol = tabs[0].replace("=", "").trim();
				String condFormula = tabs[1].trim();
				AbstractCondition cond = extractCondition(condSymbol, condFormula);
				conditions.put(condSymbol, cond);		
			}
			else if (line.trim().length() > 0) {
				throw new DialogueException("ERROR: condition file is ill-formated at line: " + i);
			}
		}
		return conditions;
	}

	
	// ==============================================================
	// CONDITION EXTRACTION METHODS
	// ==============================================================



	/**
	 * Returns a new constructed condition from a line of specification
	 * 
	 * @param content a line specifying the condition
	 * @return the constructed condition
	 * @throws DialogueException if the line is ill-formatted
	 */
	public static AbstractCondition extractCondition (String condSymbol, String str) throws DialogueException {

		str = str.trim();
		if (str.contains("[") != str.contains("]")) {
			throw new DialogueException("ERROR: bracketting is ill-formatted");
		}

		// extracting the probabilities
		float[] minmaxPrcond;
		if (str.contains("]") && !str.endsWith("]")) {
			minmaxPrcond = extractMinAndMaxProbabilities(str.split("]")[1]);
		}
		else {
			minmaxPrcond = extractMinAndMaxProbabilities(str);
		}
		debug("minmax: " + minmaxPrcond[0] + " " + minmaxPrcond[1]);
		
		// event condition
		if (str.substring(0,2).equals("E[")) {
			String eventcontent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			EventCondition cond = new EventCondition(condSymbol, eventcontent, minmaxPrcond[0], minmaxPrcond[1]);
			return cond;
		}

		// intention condition
		else if (str.substring(0,3).equals("CI[")) {
			String intentContent = str.substring(3,str.length()).split("]")[0].replace("]", "");
			IntentionCondition cond = new IntentionCondition(condSymbol, intentContent, minmaxPrcond[0], minmaxPrcond[1]);
			return cond;
		}
		
		// intention condition
		else if (str.substring(0,2).equals("I[")) {
			String intentContent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			IntentionCondition cond = new IntentionCondition(condSymbol, intentContent, minmaxPrcond[0], minmaxPrcond[1]);
			return cond;
		}

		// else, we assume it is a shallow condition
		else {
			String internalcontent = str.split("\\(")[0];
			PhonstringCondition cond = new PhonstringCondition(condSymbol, internalcontent.replace("\"", ""), minmaxPrcond[0], minmaxPrcond[1]);
			return cond;
		}
	}



	/**
	 * Extract the mininum and maximum probabilities encoded as "(min,max)" in a 
	 * specification line.  If none are given, assume a range [0,1]
	 * 
	 * @param line the line in which to extract the probabilities
	 * @return a table with the first element being the minimum probabilitiy and the second
	 *         the maximum probability
	 * @throws DialogueException if line if ill-formatted
	 */
	public static float[] extractMinAndMaxProbabilities (String line) throws DialogueException {

		if (line.contains("(") && line.contains(")") && line.indexOf("(") > line.indexOf("]")) {
			String probText = line.split("\\(")[1].replace(")", "");
			String[] split =probText.split(",");
			if (split.length == 2) {
				float minProb = Float.parseFloat(split[0].replace(",", ""));
				float maxProb = Float.parseFloat(split[1]);
				float[] result = {minProb,maxProb};
				return result;
			}
			else {
				throw new DialogueException("ERROR: wrong number of parentheses in line:" + line);
			}

		}

		float[] result = {0.0f, 1.0f};
		return result;
	}


	// ==============================================================
	// ACTION EXTRACTION METHODS
	// ==============================================================

	
	/**
	 * Extract a list of actions from a a specification text
	 * 
	 * @param actionsText the text specifying the actions
	 * @return the list of actions
	 * @throws DialogueException if the specification text is ill-formatted
	 */
	public static HashMap<String,AbstractAction> extractActions (String actionsText) throws DialogueException {

		String[] lines = actionsText.split("\n");

		HashMap<String,AbstractAction> totalActions = new HashMap<String, AbstractAction>();

		for (int i = 0 ; i < lines.length ; i++) {
			String line = lines[i];

			String[] tabs = line.split("=");

			if (tabs.length == 2) {
				String actionsSymbol = tabs[0].replace("=", "").trim();
				String content = tabs[1].trim();
				AbstractAction action = extractAction(actionsSymbol, content);
				if (action != null) {
					totalActions.put(actionsSymbol, action);
				}
			}
			else if (line.trim().length() > 0) {
				throw new DialogueException("ERROR: action file is ill-formated at line: " + i);
			}
		}
		return totalActions;
	}

	
	/**
	 * Extract an action from a line of specification
	 *  
	 * @param str the line
	 * @return the extracted action
	 * @throws DialogueException 
	 */
	public static AbstractAction extractAction (String actionSymbol, String str) throws DialogueException {
		
		str = str.trim();
		if (str.contains("[") != str.contains("]")) {
			throw new DialogueException("ERROR: bracketting is ill-formatted");
		}

		// intention action
		if (str.length() > 4 && str.substring(0,3).equals("CI[")) {
			String intentcontent = str.substring(3,str.length()).split("]")[0].replace("]", "");
			IntentionAction action = new IntentionAction(actionSymbol, FormulaUtils.constructFormula(intentcontent));
			action.setStatus(IntentionAction.COMMUNICATIVE);
			return action;
		}
		else if (str.length() > 3 && str.substring(0,2).equals("I[")) {
			String intentcontent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			IntentionAction action = new IntentionAction(actionSymbol, FormulaUtils.constructFormula(intentcontent));
			action.setStatus(IntentionAction.PRIVATE);
			return action;		}
		
		// by default, shallow dialogue action
		else {
			return new PhonstringAction(actionSymbol, str);
		}
	}



	// ==============================================================
	// METHODS FOR ATTACHING CONDITIONS AND ACTIONS TO THE POLICY
	// ==============================================================

	

	
	/**
	 * Given a policy line and lists of conditions and actions, insert a new edge,
	 * and the incoming and outgoing nodes (if they don't already exist)
	 * 
	 * @param line the policy line
	 * @param policy the dialogue policy to extend
	 * @param conditions the conditions
	 * @param actions the actions
	 * @throws DialogueException if the policy line is not well-formatted, or if the policy 
	 *         refers to actions or conditions not in the list
	 */
	public static void addEdgeAndNodesToPolicy (String line, DialoguePolicy policy, 
			HashMap<String, AbstractCondition> conditions, 
			HashMap<String, AbstractAction> actions) 
	throws DialogueException {

		String[] tabs = line.split("\t");
		if (tabs.length == 1) {
			tabs = line.split(" ");
		}
		if (tabs.length == 3) {
			String sourceNodeId = tabs[0];
			String targetNodeId = tabs[1];
			String edgeId = tabs[2];

			if (!actions.containsKey(sourceNodeId)) {
				throw new DialogueException("ERROR: " + sourceNodeId + " is not in the specified actions list");
			}
			if (!actions.containsKey(targetNodeId)) {
				throw new DialogueException("ERROR: " + targetNodeId + " is not in the specified actions list");
			}
			if (!conditions.containsKey(edgeId)) {
				throw new DialogueException("ERROR: " + edgeId + " is not in the specified conditions list");
			}

			PolicyNode sourceNode;
			if (!policy.hasNode(sourceNodeId)) {
				log("sourceNodeId: " + sourceNodeId);
				sourceNode = new PolicyNode(sourceNodeId, Arrays.asList(actions.get(sourceNodeId)));
				log("sourceNode.id: " + sourceNode.getId());
				log("hasNode: " + policy.hasNode(sourceNodeId));
				policy.addNode(sourceNode);
			}
			else {
				sourceNode = policy.getNode(sourceNodeId);
			}

			PolicyNode targetNode;
			if (!policy.hasNode(targetNodeId)  && actions.containsKey(targetNodeId)) {
				targetNode = new PolicyNode(targetNodeId, Arrays.asList(actions.get(targetNodeId)));
				policy.addNode(targetNode);
			}
			else {
				targetNode = policy.getNode(targetNodeId);
			}

			PolicyEdge newEdge = new PolicyEdge(edgeId, conditions.get(edgeId));
			newEdge.setSourceNode(sourceNode.getId());
			newEdge.setTargetNode(targetNode.getId());		
			policy.addEdge(newEdge);
		}
		else {
			throw new DialogueException("ERROR: line not well formatted");
		}
	}


	
	/**
	 * Set a node as being in the set of final nodes in the policy
	 * 
	 * @param line the line describing the final node
	 * @param policy the policy
	 * @throws DialogueException if the node cannot be set as final
	 */
	public static void addFinalNodesToPolicy (String line, DialoguePolicy policy) throws DialogueException {

		if (line.split("\t").length == 1) {

			if (policy.hasNode(line.trim())) {
				policy.setNodeAsFinal(policy.getNode(line.trim()).getId(),true);
				debug("setting node as final: " + line.trim());
			}
		}
	}



	// ==============================================================
	// UTILITY METHODS
	// ==============================================================

	
	
	/**
	 * Given a string separated by tabs or spaces, split it and take the substring
	 * at position anchor 
	 *
	 * @param line the line to split
	 * @param anchor the position in the splitted string to extract
	 * @return the extracted substring
	 */
	private static String getStringInTabbedLine (String line, int anchor) {

		if (line.contains("\t") && line.split("\t").length > anchor) {
			return line.split("\t")[anchor];
		}
		else if (line.contains(" ") && line.split(" ").length > anchor) {
			return line.split(" ")[anchor];
		}
		return "";
	}

	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[textpolicyreader] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[textpolicyreader] " + s);
		}
	}

}
