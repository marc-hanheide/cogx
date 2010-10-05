
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
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * Observation of an event, described by a formula associated with a probability
 * value (or a range of probability values)
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class EventObservation extends AbstractObservation {

	// The formula describing the event
	protected dFormula eventFormula;
	
	
	/**
	 * Create a new event observation given a propositional modal formula and a range 
	 * of possible probability values
	 * 
	 * @param eventFormula the formula describing the event
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probability
	 * @throws DialogueException if the inputs are ill-defined
	 */
	public EventObservation (dFormula eventFormula, float minProb, float maxProb) throws DialogueException {
		super(minProb,maxProb);
		if (eventFormula == null) {
			throw new DialogueException ("ERROR: formula for the intention is null");
		}
		this.eventFormula = eventFormula;
	}
	
	
	/**
	 * Create a new event observation given a propositional modal formula and a specific
	 * probability value
	 * 
	 * @param eventFormula  the formula describing the event
	 * @param prob the probability for the event
	 * @throws DialogueException 
	 */
	public EventObservation(dFormula eventFormula, float prob) throws DialogueException {
		this(eventFormula,prob,prob);
	}
	

	/**
	 * Returns the formula contained in the observation
	 * @return the formula
	 */
	public dFormula getFormula () {
		return eventFormula;
	}
	
	
	/**
	 * Returns a text representation of the event
	 */
	@Override
	public String toString() {
		return "E[" + FormulaUtils.getString(eventFormula) 
			+ "]" + " (" + minProb + ", " + maxProb + ")";
	}
	
	

	/**
	 * Returns true if the formula is underspecified, false otherwise
	 * 
	 * @return true if the formula is underspecified, false otherwise
	 */
	@Override
	public boolean isUnderspecified() {
		return (eventFormula instanceof UnderspecifiedFormula);
	}
	
	
	/**
	 * Returns true if the formula is unknown, false otherwise
	 * 
	 * @return true if the formula is unknown, false otherwise
	 */
	@Override
	public boolean isUnknown() {
		return (eventFormula instanceof UnknownFormula);
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
	public boolean equals(Object obj) {

		if (obj instanceof EventObservation && obj != null) {
			EventObservation eventObj = (EventObservation) obj;
		
				if (FormulaUtils.isEqualTo(eventFormula, eventObj.getFormula()) && 
						minProb >= eventObj.getMinProb() && maxProb <= eventObj.getMaxProb()) {
					return true;
				}
			
		}
		return false;
	}
	
}
