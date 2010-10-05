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


package de.dfki.lt.tr.dialmanagement.data.observations;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Abstract representation of an observation included a dialogue policy (at the moment, 
 * mostly a dummy class to group together the various observation types).
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public abstract class AbstractObservation {
	
	// the minimum probability
	protected float minProb;
	
	// the maximum probability
	protected float maxProb;
	

	public AbstractObservation(float minProb, float maxProb) throws DialogueException {
		if (minProb < 0.0f || minProb > 1.0f) {
			throw new DialogueException("ERROR: minProb is ill-defined");
		}
		if (maxProb < 0.0f || maxProb > 1.0f) {
			throw new DialogueException("ERROR: maxProb is ill-defined");
		}
		if (minProb > maxProb) {
			throw new DialogueException("ERROR: minProb is strictly great than maxProb");
		}
		this.minProb = minProb;
		this.maxProb = maxProb;
	}
	
	/**
	 * Returns the minimum probability
	 * @return the probability
	 */
	public float getMinProb() {
		return minProb;
	}
	
	/**
	 * Returns the maximum probability
	 * @return the probability
	 */
	public float getMaxProb() {
		return maxProb;
	}
	
	/**
	 * Changes the mimimum probability for the observation
	 * 
	 * @param minProb the mimimum probability
	 * @throws DialogueException if prob not well formed
	 */
	public void setMinProb(float minProb) throws DialogueException {
		if (minProb < 0.0f || minProb > 1.0f) {
			throw new DialogueException("ERROR: minProb is ill-defined");
		}
		this.minProb = minProb;
	}
	
	
	/**
	 * Changes the maximum probability for the observation
	 * 
	 * @param maxProb the maximum probability
	 * @throws DialogueException if prob not well formed
	 */
	public void setMaxProb(float maxProb) throws DialogueException {
		if (maxProb < 0.0f || maxProb > 1.0f) {
			throw new DialogueException("ERROR: minProb is ill-defined");
		}
		this.maxProb = maxProb;
	}
	
	
	/**
	 * Returns a text representation for the observation
	 */
	@Override
	public abstract String toString();
	
	

	/**
	 * Returns true if the formula is underspecified, false otherwise (if the observation 
	 * is not an eventObservation or an intentionObservation, always returns false)
	 * 
	 * @return false
	 */
	public boolean isUnderspecified() {
		return false;
	}
	

	/**
	 * Returns true if the formula is underspecified, false otherwise (if the observation 
	 * is not an eventObservation or an intentionObservation, always returns false)
	 * 
	 * @return false
	 */
	public boolean isUnknown() {
		return false;
	}
	
	

	/**
	 * Returns true if the current object is equivalent to the one passed as argument.
	 * By equivalency, we mean that (1) the formulae must be equal, and that (2) the 
	 * (min,max) range of probability values specified in the current object must be 
	 * contained in the (min,max) probability range of obj. 
	 * 
	 * obj the object to compare
	 * 
	 */
	@Override
	public abstract boolean equals(Object obs);
	
	
	/**
	 * Returns a hashcode for the observation
	 */
	@Override
	public int hashCode() {
		return toString().split("\\(")[0].trim().hashCode();
	}

}
