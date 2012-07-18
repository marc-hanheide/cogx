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


package de.dfki.lt.tr.dialmanagement.data.actions;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * An action which has for consequence a change in the shared dialogue state
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class DialStateAction extends AbstractAction {

	// the formula to introduce to the information state
	dFormula dialStateInfo;
	
	/**
	 * Construct a new dialogue state action given an identifier
	 * and a formula
	 * 
	 * @param id the action identifier
	 * @param dialStateInfo the formula
	 */
	public DialStateAction(String id, dFormula dialStateInfo) {
		super(id);
		this.dialStateInfo = dialStateInfo;
	}

	/**
	 * Returns the formula contained in the action
	 */
	@Override
	public dFormula asFormula() {
		return dialStateInfo;
	}

	/**
	 * Do nothing
	 */
	@Override
	public void fillArguments(HashMap<String, dFormula> arguments)
			throws DialogueException {
	}

	/**
	 * Returns an empty list
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments() {
		return new LinkedList<String>();
	}
	
	
	/**
	 * Set the formula in the dialogue state action to a particular
	 * value (replacing the previous one)
	 * 
	 * @param dialStateInfo the new formula
	 */
	public void setDialStateInfo(dFormula dialStateInfo) {
		this.dialStateInfo = dialStateInfo;
	}

	/**
	 * Returns false
	 */
	@Override
	public boolean isUnderspecified() {
		return false;
	}
	
	/**
	 * Returns a string representation of the formula
	 */
	@Override
	public String toString() {
		return FormulaUtils.getString(dialStateInfo);
	}

}
