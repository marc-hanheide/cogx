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
 
package binder.abstr;

import beliefmodels.autogen.beliefs.Belief;
import binder.arch.BindingWorkingMemory;
import binder.interfaces.BeliefWriterInterface;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
 
/**
 * 
 * NOTE: we should here add various checks to ensure the beliefs are well-formed, and maybe include
 * filters as well
 * 
 * @author plison
 *
 */
public class BeliefWriter extends ManagedComponent implements BeliefWriterInterface {

	
	/**
	 * Insert a new belief to the binder working memory
	 * 
	 * @param belief
	 * 			the belief to insert into the working memory
	 * @throws UnknownSubarchitectureException 
	 * @throws AlreadyExistsOnWMException 
	 * 			if there is already a belief with the same identifier
	 * @pre  the WM cannot include any belief with the same identifier
	 * @post belief now included into the WM
	 */
	public void insertBeliefInWM (Belief b) throws AlreadyExistsOnWMException {
		try {
		addToWorkingMemory(b.id, BindingWorkingMemory.BINDER_SA, b);
		log("new belief " + b.id + " inserted into the working memory");
		}
		catch (UnknownSubarchitectureException e) {
			log("ERROR: problem with the subarchitecture identifier for the binder");
		}
	}
	
	
	/**
	 * Update the belief already existing on the WM with a new object
	 *
	 * @param belief 
	 * 			the new belief, which must have the same identifier as the one to replace on the WM
	 * @throws ConsistencyException 
	 * @throws PermissionException 
	 * @pre the WM must include an existing belief with the same identifier
	 * @post belief on the WM is replaced
	 */  
	public void updateBeliefOnWM (Belief belief) throws DoesNotExistOnWMException, PermissionException, ConsistencyException {
		
		try {
			overwriteWorkingMemory(belief.id, BindingWorkingMemory.BINDER_SA, belief);
			log("existing belief " + belief.id + " updated on the working memory");
		} 
		catch (UnknownSubarchitectureException e) {
			log("ERROR: problem with the subarchitecture identifier for the binder");
		} 
	}
	
	
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
	public void deleteBeliefOnWM(String beliefID) throws DoesNotExistOnWMException, PermissionException {
		
		try {
			deleteFromWorkingMemory(beliefID, BindingWorkingMemory.BINDER_SA);
		}
		catch (UnknownSubarchitectureException e) {
			log("ERROR: problem with the subarchitecture identifier for the binder");
		} 
	}
}
