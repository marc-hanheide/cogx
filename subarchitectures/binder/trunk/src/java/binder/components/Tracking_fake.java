
//=================================================================                                                        
//Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                       
//This library is free software; you can redistribute it and/or                                                            
//modify it under the terms of the GNU Lesser General Public License                                                       
//as published by the Free Software Foundation; either version 2.1 of                                                      
//the License, or (at your option) any later version.                                                                      
//                                                                                                                       
//This library is distributed in the hope that it will be useful, but                                                      
//WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
//Lesser General Public License for more details.                                                                          
//                                                                                                                       
//You should have received a copy of the GNU Lesser General Public                                                         
//License along with this program; if not, write to the Free Software                                                      
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
//02111-1307, USA.                                                                                                         
//=================================================================                                                        

package binder.components;

import java.util.List;

import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.TemporalUnionBuilder;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * Dummy component for tracking (currently directly converts a multi-modal belief
 * belief into a temporal union belief)
 * 
 * @author Pierre Lison (plison@dfki.de)
 *
 */
public class Tracking_fake extends FakeComponent {
 
	    
	/**
	 * Adding change filters on working memory changes (insertion, overwrite, deletion)
	 * pertaining to multi-modal beliefs
	 * 
	 */
	@Override
	public void start() {
		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						mmBeliefAdded(_wmc);
					}
				}
		);
		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						mmBeliefUpdated(_wmc);
					}
				}
		);
		
		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						mmBeliefDeleted(_wmc);
					}
				}
		);
	}
	
	

	/**
	 * Upon the insertion of a new multi-modal belief, create one single offspring, add it to the
	 * belief offspring field, and update it
	 * 
	 * @param wmc the working memory change
	 */
	private void mmBeliefAdded (WorkingMemoryChange wmc) {
		 
		try {
			CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(wmc.address, MultiModalBelief.class);

			addOffspring(beliefData.getData(), newDataID());
			
			updateBeliefOnWM(beliefData.getData());
			
		}	

		 catch (Exception e) {
				e.printStackTrace();
			}
	}
	
	
	/**
	 * Upon the update of a given multi-modal belief, loop on its offspring.  If an offspring doesn't
	 * exist yet on the WM, create it, else update the existing one
	 * 
	 * @param wmc the working memory change
	 */
	private void mmBeliefUpdated (WorkingMemoryChange wmc) {
		
		try {

			CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(wmc.address, MultiModalBelief.class);

			log("overwriting belief " + beliefData.getID());

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
			
			log("number of offspring for belief " + beliefData.getID() + ": " + offspring.size());
			
			for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					log("belief " + child.id + " exists on WM, overwriting");
					TemporalUnionBelief childBelief = 
						TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), wmc.address, child.id);
				
					updatePointers(childBelief, TemporalUnionBelief.class);

					TemporalUnionBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), TemporalUnionBelief.class);
					childBelief.content = mergeBeliefContent(existingBelief.content, childBelief.content);
	
					updateBeliefOnWM(childBelief);
				}
				else {
					log("belief " + child.id + " does not exist on WM, creating it");
					TemporalUnionBelief childBelief = TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), wmc.address, child.id);
				
					updatePointers(childBelief, TemporalUnionBelief.class);
					
					insertBeliefInWM(childBelief);
				}
			}
			
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
	
	
	/**
	 * Upon the deletion of a given multi-modal belief, delete all the offspring of the belief
	 * 
	 * @param wmc the working memory change
	 */
	private void mmBeliefDeleted (WorkingMemoryChange wmc) {
		
		
		try {
			CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(wmc.address, MultiModalBelief.class);

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
			
			for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					deleteBeliefOnWM(child.id);
				}
			}
			
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
}
