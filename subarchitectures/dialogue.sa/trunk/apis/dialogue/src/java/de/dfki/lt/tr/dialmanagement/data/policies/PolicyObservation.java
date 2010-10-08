
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
 * Representation of a policy observation, made of an identifier,
 * a type, a content (expressed as a formula), and minimum/maximum
 * probabilities.
 * 
 * NB: the observation content is defined in FormulaWrapper
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 8/10/2010
 */
public class PolicyObservation extends FormulaWrapper {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the unique identifier for the node
	private String id;
		
	private int type;
	
	public static final int COMMUNICATIVE_INTENTION = 0;
	public static final int INTENTION = 1;
	public static final int EVENT = 2;
	
	// minimum probability for the observation
	float minProb = 0.0f;
	
	// maximum probability for the observation
	float maxProb = 1.0f;
	

	public PolicyObservation (String id) {
		super("");
		this.id = id;
	}
	
	public PolicyObservation (String id, dFormula content, float minProb, float maxProb) {
		super(content);	
		this.id = id;
		this.minProb = minProb;
		this.maxProb = maxProb;
	}
	
	
	public void setType (int type) {
		if (type == COMMUNICATIVE_INTENTION || type == INTENTION || type == EVENT) {
			this.type = type;
		}
	}
	
	public int getType() {
		return type;
	}

	public PolicyObservation (String id, dFormula content) {
		super(content);
		this.id = id;
	}
	
	public PolicyObservation (String id, String content) {
		super(content);
		this.id = id;
	}
	
	public PolicyObservation (String id, String content, float minProb, float maxProb) {
		super(content);
		this.id = id;
		this.minProb = minProb;
		this.maxProb = maxProb;
	}

	
	public boolean matchesWithObservation (Observation obs) {
		
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
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}
	

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
