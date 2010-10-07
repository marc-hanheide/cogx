
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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
import java.util.HashMap;


import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.data.PolicyAction;
import de.dfki.lt.tr.dialmanagement.data.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.PolicyNode;

/**
 * Utility for constructing a new dialogue policy from a finite-state specification
 * in the AT&T FSM format, associated with a file describing the actions and a
 * file describing the observations
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 10/06/2010
 */

public class PolicyReader {

	// logging mode
	public static boolean LOGGING = true;

	// debugging mode
	public static boolean DEBUG = false;


	/**
	 * Construct a new dialogue policy according to the specifications in the 3 configuration
	 * files (one for the policy, one for the observations, one for the actions)
	 * 
	 * @param policyFile path of the policy file
	 * @param obsFile path of the observations file
	 * @param actionsFile path of the actions file
	 * @return the constructed dialogue policy
	 * @throws DialogueException if the files don't exist or are not correctly formatted
	 */
	public static DialoguePolicy constructPolicy(String policyFile, String obsFile, 
			String actionsFile) throws DialogueException {

		try {
			// extracting the observations
			HashMap<String,PolicyEdge> observations = extractObservations (FileUtils.readfile(obsFile));

			// extracting the actions
			HashMap<String,PolicyNode> actions = extractActions (FileUtils.readfile(actionsFile));

			// constructing the policy
			String policyText = FileUtils.readfile(policyFile);
			return constructPolicy(policyText, observations, actions);

		} catch (IOException e) {
			String m = "ERROR: problem reading the files: {" + policyFile +  ", " + obsFile +
			", " + actionsFile +  "}, abording the construction of the finite-state machine";
			throw new DialogueException(m);
		}	
	}




	/**
	 * Returns a list of observations extracted from the specification text
	 * 
	 * @param obsText the text specifying the observations
	 * @return a hashmap containing the observations, indexed by their identifier
	 * @throws DialogueException if the specification text is ill-formated
	 */
	public static HashMap<String,PolicyEdge> extractObservations (String obsText) throws DialogueException {

		String[] lines = obsText.split("\n");

		HashMap<String,PolicyEdge> totalObs = new HashMap<String, PolicyEdge>();

		for (int i = 0 ; i < lines.length ; i++) {
			String line = lines[i];

			String[] tabs = line.split("=");

			if (tabs.length == 2) {
				String obsSymbol = tabs[0].replace("=", "").trim();
				String obs = tabs[1].trim();
				PolicyEdge edge = extractObservation(obs);
				edge.setId(obsSymbol);
				totalObs.put(obsSymbol, edge);		
			}
			else if (line.trim().length() > 0) {
				throw new DialogueException("ERROR: observation file is ill-formated at line: " + i);
			}
		}
		return totalObs;
	}

	

	/**
	 * Extract a list of actions from a a specification text
	 * 
	 * @param actionsText the text specifying the actions
	 * @return the list of actions
	 * @throws DialogueException if the specification text is ill-formatted
	 */
	public static HashMap<String,PolicyNode> extractActions (String actionsText) throws DialogueException {

		String[] lines = actionsText.split("\n");

		HashMap<String,PolicyNode> totalActions = new HashMap<String, PolicyNode>();

		for (int i = 0 ; i < lines.length ; i++) {
			String line = lines[i];

			String[] tabs = line.split("=");

			if (tabs.length == 2) {
				String actionsSymbol = tabs[0].replace("=", "").trim();
				String content = tabs[1].trim();
				totalActions.put(actionsSymbol, new PolicyNode(actionsSymbol, extractAction(content)));
			}
			else if (line.trim().length() > 0) {
				throw new DialogueException("ERROR: action file is ill-formated at line: " + i);
			}
		}
		return totalActions;
	}

	/**
	 * Returns a new constructed observation from a line of specification
	 * 
	 * @param content a line specifying the observation
	 * @return the constructed observation
	 * @throws DialogueException if the line is ill-formatted
	 */
	public static PolicyEdge extractObservation (String str) throws DialogueException {

		str = str.trim();
		if (str.contains("[") != str.contains("]")) {
			throw new DialogueException("ERROR: bracketting is ill-formatted");
		}

		// extracting the probabilities
		float[] minmaxProbs;
		if (str.contains("]") && !str.endsWith("]")) {
			minmaxProbs = extractMinAndMaxProbabilities(str.split("]")[1]);
		}
		else {
			minmaxProbs = extractMinAndMaxProbabilities(str);
		}
		debug("minmax: " + minmaxProbs[0] + " " + minmaxProbs[1]);
		
		// event observation
		if (str.substring(0,2).equals("E[")) {
			String eventcontent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			FormulaWrapper content = new FormulaWrapper (FormulaUtils.constructFormula(eventcontent));
			return new PolicyEdge (content, minmaxProbs[0], minmaxProbs[1]);
		}

		// intention observation
		else if (str.substring(0,3).equals("CI[")) {
			String intentContent = str.substring(3,str.length()).split("]")[0].replace("]", "");
			FormulaWrapper content = new FormulaWrapper (FormulaUtils.constructFormula(intentContent));
			return new PolicyEdge (content, minmaxProbs[0], minmaxProbs[1]);
		}
		
		// intention observation
		else if (str.substring(0,2).equals("I[")) {
			String intentContent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			FormulaWrapper content = new FormulaWrapper (FormulaUtils.constructFormula(intentContent));
			return new PolicyEdge (content, minmaxProbs[0], minmaxProbs[1]);
		}

		// else, we assume it is a shallow observation
		else {
			String internalcontent = str.split("\\(")[0];
			FormulaWrapper content = new FormulaWrapper (internalcontent.replace("\"", ""));
			return new PolicyEdge (content, minmaxProbs[0], minmaxProbs[1]);
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


	/**
	 * Extract an action from a line of specification
	 *  
	 * @param str the line
	 * @return the extracted action
	 * @throws DialogueException 
	 */
	public static PolicyAction extractAction (String str) throws DialogueException {
		
		str = str.trim();
		if (str.contains("[") != str.contains("]")) {
			throw new DialogueException("ERROR: bracketting is ill-formatted");
		}

		// intention action
		if (str.length() > 4 && str.substring(0,3).equals("CI[")) {
			String intentcontent = str.substring(3,str.length()).split("]")[0].replace("]", "");
			return new PolicyAction(intentcontent);
		}
		else if (str.length() > 3 && str.substring(0,2).equals("I[")) {
			String intentcontent = str.substring(2,str.length()).split("]")[0].replace("]", "");
			return new PolicyAction(intentcontent);
		}
		
		// by default, shallow dialogue action
		else {
			return new PolicyAction(str);
		}
	}


	/**
	 * Construct a new dialogue policy from a policy specification, a set of observations and a set of actions
	 * @param text the policy text
	 * @param observations the observations
	 * @param actions the actions
	 * @return
	 * @throws DialogueException if the policy text is not well formatted
	 */
	public static DialoguePolicy constructPolicy (String text, HashMap<String, PolicyEdge> observations, 
			HashMap<String, PolicyNode> actions) throws DialogueException {

		String[] lines = text.split("\n");

		DialoguePolicy policy = new DialoguePolicy();

		for (int i = 0 ; i < lines.length ; i++) {
			String curLine = lines[i];
			debug("parsing line: " + curLine);
			
			// line formatted as "inNode outNode observation"
			if ((curLine.split("\t").length == 3) || (curLine.split(" ").length == 3)) {
				addEdgeAndNodesToPolicy (curLine, policy, observations, actions);
				
				// if it is the first line, set the first node as initial
				if (i == 0) {
					debug("init node: " + getStringInTabbedLine(curLine,0));
					policy.setNodeAsInitial(policy.getNode(getStringInTabbedLine(curLine, 0)));
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
	 * Given a policy line and lists of observations and actions, insert a new edge,
	 * and the incoming and outgoing nodes (if they don't already exist)
	 * 
	 * @param line the policy line
	 * @param policy the dialogue policy to extend
	 * @param observations the observations
	 * @param actions the actions
	 * @throws DialogueException if the policy line is not well-formatted, or if the policy 
	 *         refers to actions or observations not in the list
	 */
	public static void addEdgeAndNodesToPolicy (String line, DialoguePolicy policy, 
			HashMap<String, PolicyEdge> observations, 
			HashMap<String, PolicyNode> actions) 
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
			if (!observations.containsKey(edgeId)) {
				throw new DialogueException("ERROR: " + edgeId + " is not in the specified observations list");
			}

			PolicyNode sourceNode;
			if (!policy.hasNode(sourceNodeId)) {
				sourceNode = policy.addNode(sourceNodeId, actions.get(sourceNodeId).getAction());
			}
			else {
				sourceNode = policy.getNode(sourceNodeId);
			}

			PolicyNode targetNode;
			if (!policy.hasNode(targetNodeId)  && actions.containsKey(targetNodeId)) {
				targetNode = policy.addNode(targetNodeId, actions.get(targetNodeId).getAction());
			}
			else {
				targetNode = policy.getNode(targetNodeId);
			}

			PolicyEdge newEdge = observations.get(edgeId).copy();
			newEdge.setSourceNode(sourceNode);
			newEdge.setTargetNode(targetNode);		
			policy.addEdge(newEdge, sourceNode, targetNode);
		}
		else {
			throw new DialogueException("ERROR: line not well formatted");
		}
	}


	/**
	 * Set a node as being in the set of final nodes in the policy
	 * @param line the line describing the final node
	 * @param policy the policy
	 * @throws DialogueException if the node cannot be set as final
	 */
	public static void addFinalNodesToPolicy (String line, DialoguePolicy policy) throws DialogueException {

		if (line.split("\t").length == 1) {

			if (policy.hasNode(line.trim())) {
				policy.setNodeAsFinal(policy.getNode(line.trim()));
				debug("setting node as final: " + line.trim());
			}
		}
	}

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[Dialogue policy] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[Dialogue policy] " + s);
		}
	}

}
