
// =================================================================                                                        
// Copyright (C) 2010-2012 Pierre Lison (plison@dfki.de)                                                                
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


package binder.components;

/**
 * Dummy CAST component, to verify all compilation operations
 * work correctly
 * 
 * @author Pierre Lison
 * @version 13/03/2010
 * @started 13/03/2010
 */

import cast.architecture.ManagedComponent;

public class DummyComponent extends ManagedComponent {

	public boolean LOGGING = true;
	
	
	public void start() {
		log("OK, dummy is working !");
	}
	
	
	
	 
	private void log(String s) {
		if (LOGGING) {
		System.out.println("[Dummy] " + s);
		}
	}
}
