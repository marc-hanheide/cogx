
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
 

package binder.interfaces;


import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.ProbDistribution;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.cdl.WorkingMemoryPointer;


public interface BeliefWriterInterface {

	
	/**
	 * Insert a new belief to the binder working memory
	 * 
	 * @param belief
	 * 			the belief to insert into the working memory
	 * @throws AlreadyExistsOnWMException 
	 * @pre  the WM cannot include any belief with the same identifier
	 * @post belief now included into the WM
	 */
	public void insertBeliefInWM (Belief belief) throws AlreadyExistsOnWMException;
	
	
	
	/**
	 * Update the belief already existing on the WM with a new object
	 *
	 * @param belief 
	 * 			the new belief, which must have the same identifier as the one to replace on the WM
	 * @throws DoesNotExistOnWMException 
	 * @throws ConsistencyException 
	 * @throws PermissionException 
	 * @pre the WM must include an existing belief with the same identifier
	 * @post belief on the WM is replaced
	 */ 
	public void updateBeliefOnWM (Belief belief) throws DoesNotExistOnWMException, PermissionException, ConsistencyException;
	
	
	
	/**
	 * Remove a belief from the WM
	 * 
	 * @param beliefID 
	 * 			the identifier of the belief to remove from the WM
	 * @throws PermissionException 
	 * @throws DoesNotExistOnWMException 
	 * @pre the WM must include a belief with the given identifier
	 * @post the belief is removed
	 */
	public void deleteBeliefOnWM(String beliefID) throws DoesNotExistOnWMException, PermissionException;
	
}
