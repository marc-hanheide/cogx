
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


import cast.cdl.WorkingMemoryPointer;
import binder.autogen.beliefs.Belief;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.ProbDistribution;


public interface PerceptWriterInterface {

	
	/**
	 * Construct a WorkingMemoryPointer object (information about the proxy origin:
	 * subarchitecture identifier, local data ID in subarchitecture, and data type)
	 * 
	 * @param subarchId
	 *            subarchitecture identifier
	 * @param localDataId
	 *            identifier of the data in local subarchitecture
	 * @param localDataType
	 *            type of the data in local subarchitecture
	 * @return a new WorkingMemoryPointer object
	 */
	public WorkingMemoryPointer createWorkingMemoryPointer (String subarchId, String localDataId, 
			String localDataType) ;

	
	/** 
	 * Create a new percept belief object, with a new forged identifier, private
	 * epistemic status, current spatio-temporal frame, empty belief content, and 
	 * history defined the "origin" pointer to the local data structure
	 * 
	 * @param origin 
	 * 			a working memory pointer to the local data structure which is at
	 * 			the origin of the percept
	 * @return a new percept belief with the specs defined above
	 * 
	 */
	public PerceptBelief createNewPercept(WorkingMemoryPointer origin);
	
	
	/** 
	 * Create a new percept belief object, with a existing identifier, private
	 * epistemic status, current spatio-temporal frame, empty belief content, and 
	 * history defined the "origin" pointer to the local data structure
	 * 
	 * @param existingID 
	 * 			an identifier for a pre-existing percept belief
	 * @param origin 
	 * 			a working memory pointer to the local data structure which is at
	 *			the origin of the percept
	 * @return a new percept belief with the specs defined above
	 * 
	 */	
	public PerceptBelief createNewPercept(String existingID, WorkingMemoryPointer origin);
	
	
	/**
	 * Add a belief content to an existing belief
	 * 
	 * @param percept the existing belief
	 * @param content the belief content, specified as a probability distribution
	 * @post percept now includes the given belief content
	 */
	public void addBeliefContent(PerceptBelief percept, ProbDistribution content);
	
	
	/**
	 * Insert a new belief to the binder working memory
	 * 
	 * @param belief
	 * 			the belief to insert into the working memory
	 * @pre  the WM cannot include any belief with the same identifier
	 * @post belief now included into the WM
	 */
	public void insertBeliefIntoWM (Belief belief);
	
	
	
	/**
	 * Update the belief already existing on the WM with a new object
	 *
	 * @param belief 
	 * 			the new belief, which must have the same identifier as the one to replace on the WM
	 * @pre the WM must include an existing belief with the same identifier
	 * @post belief on the WM is replaced
	 */ 
	public void updateBeliefOnWM (Belief belief);
	
	
	/**
	 * Remove a belief from the WM
	 * 
	 * @param beliefID 
	 * 			the identifier of the belief to remove from the WM
	 * @pre the WM must include a belief with the given identifier
	 * @post the belief is removed
	 */
	public void deleteBeliefONWM(String beliefID);
}
