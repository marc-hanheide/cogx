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


package de.dfki.lt.tr.dialmanagement.data.actions;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * Intention action, encapsulating a private intention which still needs to be realised
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class IntentionAction extends AbstractAction {

	// The intention
	dFormula intentFormula;
	
	/**
	 * Construct a new intention action based on a formula
	 * 
	 * @param utterance the utterance string
	 */
	public IntentionAction (dFormula intentFormula) {
		this.intentFormula = intentFormula;
	}
	
	/**
	 * Returns the utterance
	 */
	@Override
	public String toString() {
		return "I[" + FormulaUtils.getString(intentFormula) + "]";
	}
	
	
	public dFormula getFormula() {
		return intentFormula;
	}

	/**
	 * Returns true if the current object is equivalent to the one passed as argument.
	 * By equivalency, we mean that the content of both actions must be equal
	 * 
	 * obj the object to compare
	 * 
	 */
	@Override
	public  boolean equals(Object obj) {
		if (obj != null && obj instanceof IntentionAction) {
			return (FormulaUtils.isEqualTo(intentFormula, ((IntentionAction)obj).getFormula()));
		}
		return false;
	}
	
}
