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


/**
 * Abstract representation of an action included a dialogue policy (at the moment, 
 * only a dummy class to group together the various action types).
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public abstract class AbstractAction {

	
	/**
	 * Returns true if the current object is equivalent to the one passed as argument.
	 * By equivalency, we mean that the content of both actions must be equal
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
		return toString().hashCode();
	}
}

