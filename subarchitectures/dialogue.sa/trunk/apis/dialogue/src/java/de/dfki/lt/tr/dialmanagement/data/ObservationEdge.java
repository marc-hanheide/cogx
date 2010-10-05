
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

package de.dfki.lt.tr.dialmanagement.data;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.observations.Observation;
import de.dfki.lt.tr.dialmanagement.data.observations.ObservationContent;

/**
 * Observation edge in a dialogue policy
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 10/06/2010
 */

public class ObservationEdge {


	// logging mode
	public static boolean LOGGING = true;

	// debugging mode
	public static boolean DEBUG = true;
	
	// the (unique) identifier for the edge
	private String id;
	
	// Incoming (=origin) action node
	private ActionNode in;
	
	// Outgoing (=destination) action node
	private ActionNode out;
	
	// Observation content contained in the edge
	private ObservationContent content;
	
	// minimum probability for the observation
	float minProb;
	
	// maximum probability for the observation
	float maxProb;
	
	/**
	 * Constructs a new observation edge from an incoming action node, an outgoing action node, and an observation
	 * 
	 * @param id the edge identifier
	 * @param in the incoming node
	 * @param out the outgoing node
	 * @param obs the observation
	 * @param the minimum probability
	 * @param the maximum probability
	 * @throws DialogueException if one of the parameters is a null value
	 */
	public ObservationEdge (String id, ActionNode in, ActionNode out, ObservationContent obs, float minProb, float maxProb) throws DialogueException {
		
		this.minProb = minProb;
		this.maxProb = maxProb;
		
		if (id != null && in != null && out != null && obs != null) {
			this.id = id;
			this.in = in;
			this.out = out;
			this.content = obs;
		}
		else {
			throw new DialogueException("ERROR: cannot enter null values in observation edge");
		}
	}
	
	
	public ObservationEdge (ObservationContent obs, float minProb, float maxProb) {
		
		this.minProb = minProb;
		this.maxProb = maxProb;
		this.content = obs;
	}
	
	public ObservationEdge copy () throws DialogueException {
		return new ObservationEdge(content,minProb,maxProb);
	}
	
	
	public void setId(String id) {
		this.id = id;
	}
	
	public void setSourceNode(ActionNode in) {
		this.in = in;
	}
	
	public void setTargetNode(ActionNode out) {
		this.out = out;
	}
	
	
	/**
	 * Returns the incoming action node (i.e. the origin of the edge)
	 * @return the incoming action node
	 */
	public ActionNode getIncomingAction() {
		return in;
	}
	
	/**
	 * Returns the outgoing action node (i.e. the destination of the edge)
	 * @return the outgoing action node
	 */
	public ActionNode getOutgoingAction() {
		return out;
	}
	
	/**
	 * Returns the observation contained in the edge
	 * @return the observation
	 */
	public ObservationContent getObservation() {
		return content;
	}
	
	/**
	 * Set the incoming action node of the edge (overriding the existing one)
	 * 
	 * @param in the new incoming action node
	 * @throws DialogueException if the action node is a null value
	 */
	public void setIncomingAction(ActionNode in) throws DialogueException {
		if (in != null) {
			this.in = in;
		}
		else {
			throw new DialogueException("ERROR: cannot enter null values in observation edge");
		}
	}
	
	/**
	 * Set the outgoing action node of the edge (overriding the existing one)
	 * 
	 * @param in the new outgoing action node
	 * @throws DialogueException if the action node is a null value
	 */
	public void setOutgoingAction(ActionNode out) throws DialogueException {
		if (out != null) {
			this.out = out;
		}
		else {
			throw new DialogueException("ERROR: cannot enter null values in observation edge");
		}
	}
	
	/**
	 * Set the observation contained in the edge (overriding the existing one)
	 * 
	 * @param obs the new observation
	 * @throws DialogueException if the observation is a null value
	 */
	public void setObservation(ObservationContent obs) throws DialogueException {
		if (obs != null) {
			this.content = obs;
		}
		else {
			throw new DialogueException("ERROR: cannot enter null values in observation edge");
		}
	}
	
	
	public boolean matchesWithObservation (Observation obs) {
		
		for (ObservationContent alternative : obs.getAlternatives()) {
			if (alternative.equals(content) && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {				
				return true;
			}
			else if (obs.getProbability(alternative) == 0.8f){
				debug("alternative: " + alternative.toString());
				debug("content: " + content);
				debug("prob: " + obs.getProbability(alternative));
				debug ("min: " + minProb);
				debug("max: " + maxProb);
			}
		}
		return false;
	}
	
	
	public String toString () {
		return content.toString() + " (" + minProb + ", " + maxProb + ")";
	}
	
	
	/**
	 * Returns the edge identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
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
