
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

import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.StableBeliefBuilder;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Dummy component for temporal smoothing (currently directly converts a temporal union belief
 * belief into a stable belief)
 * 
 * @author Pierre Lison (plison@dfki.de)
 *
 */

public class TemporalSmoothing_fake extends FakeComponent {


	/**
	 * Adding change filters on working memory changes (insertion, overwrite, deletion)
	 * pertaining to temporal union beliefs
	 * 
	 */
	@Override
	public void start() {
		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						tunionBeliefAdded(_wmc);
					}
				}
		);
		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						tunionBeliefUpdated(_wmc);
					}
				}
		);
		
		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						tunionBeliefDeleted(_wmc);
					}
				}
		);
	}
	


	/**
	 * Upon the insertion of a new temporal union belief, create one single offspring, add it to the
	 * belief offspring field, and update it
	 * 
	 * @param wmc the working memory change
	 */
	private void tunionBeliefAdded (WorkingMemoryChange wmc) {
		 
		try {
			CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, TemporalUnionBelief.class);
			
			addOffspring(beliefData.getData(), newDataID());	
			updateBeliefOnWM(beliefData.getData());
				
		}	

		 catch (Exception e) {
				e.printStackTrace();
			}
	}
	
	
	/**
	 * Upon the update of a given temporal union belief, loop on its offspring.  If an offspring doesn't
	 * exist yet on the WM, create it, else update the existing one
	 * 
	 * @param wmc the working memory change
	 */
	private void tunionBeliefUpdated (WorkingMemoryChange wmc) {
	
		try {
			CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, TemporalUnionBelief.class);

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
			log("number of offspring for : " + beliefData.getData().id + ": "+ offspring.size());

			for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					log("belief " + child.id + " exists on WM, merging content");
					StableBelief childBelief =StableBeliefBuilder.createnewStableBelief(beliefData.getData(), wmc.address, child.id);

					updatePointers(childBelief, StableBelief.class);

					StableBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), StableBelief.class);
					childBelief.content = mergeBeliefContent(existingBelief.content, childBelief.content);
					
					updateBeliefOnWM(childBelief);
				}
				else {
					log("belief " + child.id + " does not exist on WM, creating it");
					StableBelief childBelief =StableBeliefBuilder.createnewStableBelief(beliefData.getData(), wmc.address, child.id);
					updatePointers(childBelief, StableBelief.class);
					insertBeliefInWM(childBelief);
				}
			}
			
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
	
	
	/**
	 * Upon the deletion of a given temporal union belief, delete all the offspring of the belief
	 * 
	 * @param wmc the working memory change
	 */
	private void tunionBeliefDeleted (WorkingMemoryChange wmc) {
		
		try {
			CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, TemporalUnionBelief.class);

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
