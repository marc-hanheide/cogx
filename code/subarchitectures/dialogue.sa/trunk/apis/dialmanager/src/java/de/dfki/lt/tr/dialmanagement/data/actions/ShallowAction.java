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
 * Shallow dialogue action, encapsulating a fully realised utterance string
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class ShallowAction extends AbstractAction {

	// The utterance
	String utterance;
	
	/**
	 * Construct a new shallow dialogue action, based on an utterance string
	 * 
	 * @param utterance the utterance string
	 */
	public ShallowAction (String utterance) {
		this.utterance = utterance;
	}
	
	/**
	 * Returns a textual representation of the action
	 */
	@Override
	public String toString() {
		return utterance;
	}
	
	/**
	 * Returns the utterance
	 */
	public String getUtterance() {
		return utterance;
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
		if (obj != null && obj instanceof ShallowAction) {
			return (utterance.equals(((ShallowAction)obj).getUtterance())) ;
		}
		return false;
	}
}
