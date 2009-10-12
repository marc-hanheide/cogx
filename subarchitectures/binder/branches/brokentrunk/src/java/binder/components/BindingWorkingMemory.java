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


package binder.components;

import cast.architecture.SubarchitectureWorkingMemory;

/** 
 * The binder working memory (currently just an empty wrapper around
 * SubarchitectureWorkingMemory)
 * 
 * TODO: extend this wrapper to record binding history (to be used to navigate in the binding states)
 * TODO:  + maybe some forgetting mechanism
 * TODO: + mechanisms to import/export BindingWorkingMemory states
 * 
 * @author Pierre Lison
 * @version 31/08/2009
 */

public class BindingWorkingMemory extends SubarchitectureWorkingMemory {

}
