
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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * Representation of a policy condition on possible observations.  A policy
 * condition is defined by an identifier, a type, a content (expressed as 
 * a formula), and minimum/maximum probabilities.
 * 
 * NB: the condition content is defined in FormulaWrapper
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 8/10/2010
 */
public class PolicyCondition extends FormulaWrapper {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the unique identifier for the node
	private String id;
	
	// the condition type
	private int type;
	
	// allowed condition types
	public static final int COMMUNICATIVE_INTENTION = 0;
	public static final int INTENTION = 1;
	public static final int EVENT = 2;
	
	// minimum probability for the condition
	float minProb = 0.0f;
	
	// maximum probability for the condition
	float maxProb = 1.0f;
	

	/**
	 * Create a new policy condition given an identifier
	 * 
	 * @param id the identifier
	 */
	public PolicyCondition (String id) {
		super("");
		this.id = id;
	}
	
	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the condition content
	 */
	public PolicyCondition (String id, dFormula content) {
		this(id,content,0.0f,1.0f);
	}
	
	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the condition content
	 */
	public PolicyCondition (String id, String content) {
		this(id,content,0.0f, 1.0f);
	}

	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the formula content
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public PolicyCondition (String id, dFormula content, float minProb, float maxProb) {
		super(content);	
		this.id = id;
		this.minProb = minProb;
		this.maxProb = maxProb;
	}
	
	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the formula content
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public PolicyCondition (String id, String content, float minProb, float maxProb) {
		super(content);
		this.id = id;
		this.minProb = minProb;
		this.maxProb = maxProb;
	}
	
	/**
	 * Sets the type of condition (within the set of allowed ones, cf. above)
	 * 
	 * @param type the condition type
	 */
	public void setType (int type) {
		if (type == COMMUNICATIVE_INTENTION || type == INTENTION || type == EVENT) {
			this.type = type;
		}
	}
	
	/**
	 * Returns the type of condition
	 * @return
	 */
	public int getType() {
		return type;
	}


	/**
	 * Returns true if the policy condition matches a given observation
	 * 
	 * @param obs the observation
	 * @return true if a match is found, false otherwise
	 */
	public boolean matchesObservation (Observation obs) {
		
		for (FormulaWrapper alternative : obs.getAlternatives()) {
			if (alternative.equals(this) && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {				
				return true;
			}
		}
		return false;
	}
	

	/**
	 * Returns the node identifier
	 * 
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}
	
	/**
	 * Returns a string representation of the policy condition
	 */
	@Override
	public String toString () {
		String typeStr = "";
		if (type == COMMUNICATIVE_INTENTION) {
			typeStr += "CI[";
		}
		else if (type == INTENTION) {
			typeStr += "I[";
		}
		else if (type == EVENT) {
			typeStr += "E[";
		}
		
		return typeStr + FormulaUtils.getString(content) + " (" + minProb + ", " + maxProb + ")" + "]";
	}
}
