
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

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;


/**
 * Observation of a shallow dialogue observation, described by an utterance string
 * associated with a probability value (or a range of probability values)
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class ShallowObservation extends AbstractObservation {

	// the utterance
	protected String utterance;
	
	/**
	 * Create a new shallow observation based on an utterance string and a range of 
	 * probability values
	 * 
	 * @param utterance the utterance string
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probability
	 * @throws DialogueException if the inputs are ill-defined
	 */
	public ShallowObservation (String utterance, float minProb, float maxProb) throws DialogueException {
		super(minProb,maxProb);
		if (utterance == null) {
			throw new DialogueException ("ERROR: utterance is null");
		}
		this.utterance = utterance;
	}
	
	
	/**
	 * Create a new shallow observation based on an utterance string and a specific
	 * probability value
	 * 
	 * @param utterance the utterance
	 * @param prob the probability value
	 * @throws DialogueException if the inputs are ill-defined
	 */
	public ShallowObservation(String utterance, float prob) throws DialogueException {
		this(utterance, prob, prob);
	}
 
	
	/**
	 * Returns a textual representation of the observation
	 */
	public String toString() {
		return utterance + " (" + minProb + ", " + maxProb + ")";
	}
	
	
	/**
	 * Returns the utterance included inside the observation
	 * @return the utterance string
	 */
	public String getUtterance() {
		return utterance;
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
	public boolean equals(Object obs) {
		
		 if (obs != null && obs instanceof ShallowObservation) {
			 
			if	 (utterance.equals(((ShallowObservation)obs).utterance)
				 && (minProb >= ((ShallowObservation)obs).getMinProb()) &&
				 (maxProb <= ((ShallowObservation)obs).getMaxProb())) {
			 return true;
		 }
		 }
		 return false;
	}
	
}
