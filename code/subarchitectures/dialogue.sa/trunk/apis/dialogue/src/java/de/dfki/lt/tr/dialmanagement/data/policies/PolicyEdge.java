
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

package de.dfki.lt.tr.dialmanagement.data.policies;

import de.dfki.lt.tr.dialmanagement.data.Observation;

/**
 * Representation of an edge in the dialogue policy.  An edge is made of
 * four elements: 
 * (1) an identifier, 
 * (2) a source node, 
 * (3) a target node,
 * and (4) an associated observation.
 * 
 * TODO: have a method to verify that the edge if complete
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 8/10/2010
 */

public class PolicyEdge {
	
	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
		
	// the (unique) identifier for the edge
	private String id;
	
	// the source node
	private PolicyNode in;
	
	// the target node
	private PolicyNode out;

	// the observation associated with the edge
	private PolicyObservation obs;
	
	
	/**
	 * Constructs a new edge from an identifier, a source node, a target node, and an observation
	 * 
	 * @param id the edge identifier
	 * @param in the source node
	 * @param out the target node
	 * @param obs the observation
	 */
	public PolicyEdge (String id, PolicyNode in, PolicyNode out, PolicyObservation obs)  {			
		this.id = id;
		this.in = in;
		this.out = out;
		this.obs = obs;
	}
	
	/**
	 * Constructs a new edge with an identifier, a source node and a target node
	 * 
	 * @param id the edge identifier
	 * @param in the source node
	 * @param out the target node
	 */
	public PolicyEdge (String id, PolicyNode in, PolicyNode out) {
		this(id,in,out,null);
	}
	
	/**
	 * Constructs a new edge with an identifier and an observation
	 * 
	 * @param id the edge identifier
	 * @param obs the observation
	 */
	public PolicyEdge (String id, PolicyObservation obs){
		this(id,null,null,obs);
	}
		
	/**
	 * Returns a copy of the edge
	 * 
	 * @return the edge copy
	 */
	public PolicyEdge copy () {
		return new PolicyEdge(id, in, out, obs);
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
	public void setSourceNode(PolicyNode in) {
		this.in = in;
	}
	
	/**
	 * Set the target node for the edge
	 * 
	 * @param out
	 */
	public void setTargetNode(PolicyNode out) {
		this.out = out;
	}
	
	
	/**
	 * Sets the policy observation for the edge
	 * 
	 * @param obs the observation
	 */
	public void setObservation(PolicyObservation obs) {
		this.obs = obs;
	}
	
	/**
	 * Returns the incoming action node (i.e. the origin of the edge)
	 * @return the incoming action node
	 */
	public PolicyNode getIncomingAction() {
		return in;
	}
	
	
	/**
	 * Returns the source node
	 * @return the source node
	 */
	public PolicyNode getSourceNode() {
		return in;
	}
	
	/**
	 * Returns the target node
	 * @return the target node
	 */
	public PolicyNode getTargetNode() {
		return out;
	}
	
	
	/**
	 * Returns the observation contained in the edge
	 * @return the observation
	 */
	public PolicyObservation getObservation() {
		return obs;
	}
	
	
	/**
	 * Returns true if the policy observation contained in the edge
	 * matches the full observation passed as argument
	 * 
	 * @param fullObs the full, runtime observation
	 * @return true if the two observations match, false otherwise
	 */
	public boolean matchesWithObservation (Observation fullObs) {
		return obs.matchesWithObservation(fullObs);
	}
	
		
	/**
	 * Returns the edge identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}
	
	/**
	 * Returns a string representation of the edge
	 */
	public String toString() {
		if (obs != null) {
		return obs.toString();
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
