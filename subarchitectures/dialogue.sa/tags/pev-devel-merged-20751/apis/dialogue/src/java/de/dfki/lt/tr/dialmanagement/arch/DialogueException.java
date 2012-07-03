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

package de.dfki.lt.tr.dialmanagement.arch;
  
    
/**
 * Exception in the dialogue management subsystem
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 3/07/2010
 *
 */
public class DialogueException extends Exception{
 
	private static final long serialVersionUID = 1L;
	
	// the message contained in the exception
	private String message = "";
	
	/**
	 * Create a new dialogue exception, with a particular message
	 * 
	 * @param message the message string
	 */
	public DialogueException (String message) {
		this.message = message;
	}

	/**
	 * Returns the message contained in the exception
	 */
	public String getMessage() {
		return message;
	}

	
	/**
	 * Returns the message contained in the exception
	 */
	@Override
	public String toString() {
		return message;
	}
}
