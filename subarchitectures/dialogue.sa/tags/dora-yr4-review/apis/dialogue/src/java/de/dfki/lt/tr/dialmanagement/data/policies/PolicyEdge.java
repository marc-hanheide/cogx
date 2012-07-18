
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

package de.dfki.lt.tr.dialmanagement.data.policies;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;

/**
 * Representation of an edge in the dialogue policy.  An edge is made of
 * four elements: 
 * (1) an identifier, 
 * (2) a source node, 
 * (3) a target node,
 * and (4) an associated condition on possible observations.
 *  
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 */

public class PolicyEdge {
	
	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
		
	// the (unique) identifier for the edge
	private String id;
	
	// the identifier of the source node
	private String in;
	
	// the identifier of the target node
	private String out;

	
	// the conditions associated with the edge
	List<AbstractCondition> conditions = new LinkedList<AbstractCondition>();

	
	// ==============================================================
	// EDGE CONSTRUCTION METHODS
	// ==============================================================

	
	/**
	 * Constructs a new edge from an identifier, a source node, a target node, and an condition
	 * 
	 * @param id the edge identifier
	 * @param in the source node
	 * @param out the target node
	 * @param condition the condition
	 */
	public PolicyEdge (String id, String in, String out, AbstractCondition condition)  {			
		this.id = id;
		this.in = in;
		this.out = out;
		if (condition != null) {
			conditions.add(condition);
		}
	}
	
	
	
	/**
	 * Constructs a new edge with an identifier, a source node and a target node
	 * 
	 * @param id the edge identifier
	 * @param in the source node
	 * @param out the target node
	 */
	public PolicyEdge (String id, String in, String out) {
		this(id,in,out,null);
	}
	
	/**
	 * Constructs a new edge with an identifier and an condition
	 * 
	 * @param id the edge identifier
	 * @param condition the condition
	 */
	public PolicyEdge (String id, AbstractCondition condition){
		this(id,null,null,condition);
	}
	
	
	/**
	 * Set a new identifier to the edge
	 * 
	 * @param id the new identifier
	 */
	public void setId(String id) {
		this.id = id;
	}
	
	/**
	 * Set the source node for the edge
	 * 
	 * @param in the source node
	 */
	public void setSourceNode(String in) {
		this.in = in;
	}
	
	/**
	 * Set the target node for the edge
	 * 
	 * @param out
	 */
	public void setTargetNode(String out) {
		this.out = out;
	}
	
	
	/**
	 * Add a policy condition for the edge
	 * 
	 * @param condition the condition
	 */
	public void addCondition(AbstractCondition condition) {
		conditions.add(condition);
	}
	

	/**
	 * Remove a policy condition to the edge
	 * 
	 * @param cond the condition to remove
	 */
	public void removeCondition(AbstractCondition cond) {
		conditions.remove(cond);
	}
	
	

	/**
	 * Replace a policy condition by another one
	 * 
	 * @param condition the old condition to be replaced
	 * @param pcond the new one
	 */
	public void replaceCondition(AbstractCondition condition, AbstractCondition pcond) {
		int index = conditions.indexOf(condition);
		conditions.set(conditions.indexOf(condition),pcond);
		log("new element in the condition: " + conditions.get(index));
	}

	
	
	// ==============================================================
	// GETTER METHODS
	// ==============================================================

	
	
	/**
	 * Returns the source node
	 * @return the source node
	 */
	public String getSourceNodeId() {
		return in;
	}
	
	/**
	 * Returns the target node
	 * @return the target node
	 */
	public String getTargetNodeId() {
		return out;
	}
	

	
	/**
	 * Returns the conditions contained in the edge
	 * @return the condition
	 */
	public List<AbstractCondition> getConditions() {
		return conditions;
	}
	
	
	/**
	 * Returns the conditions as one single formula
	 * 
	 * @return a formula comprising all the conditions
	 */
	public dFormula getConditionsAsSingleFormula() {
		if (conditions.size() == 0) {
			return new UnknownFormula(0);
		}
		else if (conditions.size() == 1) {
			return conditions.get(0).asFormula();
		}
		else {
			ComplexFormula cform = new ComplexFormula();
			cform.op = BinaryOp.conj;
			List<dFormula> formList = new LinkedList<dFormula>();
			for (AbstractCondition cond: conditions) {
				formList.add(cond.asFormula());
			}
			cform.forms = formList;
			return cform;
		}
	}
	
	/**
	 * Get the list of all underspecified variables for each
	 * condition associated to the edge
	 * 
	 * @return the list of underspecified variables
	 */
	public Collection<String> getAllUnderspecifiedArguments() {
		Collection<String> arguments = new LinkedList<String>();
		for (AbstractCondition cond: conditions) {
			arguments.addAll(cond.getUnderspecifiedArguments());
		}
		return arguments;
	}
	
	/**
	 * Returns true if the policy condition contained in the edge
	 * matches the provided observation
	 * 
	 * @param obs the observation
	 * @return true if the condition and observation match, false otherwise
	 */
	public boolean matchAllConditions (Observation obs, DialogueState dialState) {
		for (AbstractCondition condition : conditions) {
			if (!condition.matchCondition(obs, dialState)) {
				return false;
			}
		}
		return true;
	}
	
		
	/**
	 * Returns the edge identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}
	
	
	
	// ==============================================================
	// UTILITY METHODS
	// ==============================================================

	
	
	/**
	 * Returns true is the edge is complete and well-formed, false otherwise
	 * 
	 * @return true if well-formed, else false
	 */
	public boolean isWellFormed() {
		return (in != null && out != null && conditions != null && conditions.size() > 0 && id != null);
	}
	
	
	/**
	 * Returns a copy of the edge
	 * 
	 * @return the edge copy
	 */
	public PolicyEdge copy () {
		PolicyEdge edge = new PolicyEdge(id, in, out);
		for (AbstractCondition condition : conditions) {
			edge.addCondition(condition);
		}
		return edge;
	}
	
	
	/**
	 * Returns a string representation of the edge
	 */
	public String toString() {
		if (conditions != null) {
		return conditions.toString();
		}
		else {
			return "voidEdge";
		}
	}
	
	 
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[policyedge] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[policyedge] " + s);
		}
	}





} 
