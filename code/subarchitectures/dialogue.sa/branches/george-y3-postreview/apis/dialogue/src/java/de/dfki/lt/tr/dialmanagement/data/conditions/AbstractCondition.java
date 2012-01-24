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

package de.dfki.lt.tr.dialmanagement.data.conditions;

import java.util.Collection;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;

/**
 * Abstract class providing generic functionalities available for all
 * kinds of conditions
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 21/12/2010
 *
 */
public abstract class AbstractCondition {
	
	// the unique identifier for the node
	protected String id;
 
	// minimum probability for the condition
	protected float minProb = 0.0f;

	// maximum probability for the condition
	protected float maxProb = 1.0f;

	
	
	/**
	 * Create a new policy condition given an identifier
	 * 
	 * @param id the identifier
	 */
	public AbstractCondition (String id) {
		this.id = id;
	}

	/**
	 * Returns the underspecified subformulae contained in the condition
	 * 
	 * @return a collection of underspecified subformulae
	 */
	public Collection<String> getUnderspecifiedArguments () {
		return new LinkedList<String>();
	}
	

	/**
	 * Returns true if the condition is matched when given a particular observation
	 * and a dialogue state instance, false otherwise
	 * 
	 * (Abstract method that every concrete class must implement)
	 * @param obs the observation
	 * @param dialState the dialogue state
	 * @return true if the condition matches the input, false otherwise
	 */
	public abstract boolean matchCondition (Observation obs, DialogueState dialState);
	
	
	/**
	 * Returns a translation of the condition content as a logical formula
	 * 
	 * (Abstract method that every concrete class must implement)
	 * @return the method content represented as a formula
	 */
	public abstract dFormula asFormula();
	


	/**
	 * Returns the node identifier
	 * 
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}


	/**
	 * Returns the minimum probability for the condition
	 * 
	 * @return min probability
	 */
	public float getMinimumProb () {
		return minProb;
	}

	/**
	 * Returns the maximum probability for the condition
	 * 
	 * @return max probability
	 */
	public float getMaximumProb() {
		return maxProb;
	}

	/**
	 * Change the minimum probability for the condition
	 * 
	 * @param minProb the minimum probability
	 */
	public void setMinimumProb(float minProb) {
		this.minProb = minProb;
	}


	/**
	 * Change the maximum probability for the condition
	 * 
	 * @param maxProb the maximum probability
	 */
	public void setMaximumProb(float maxProb) {
		this.maxProb = maxProb;
	}



	
}
