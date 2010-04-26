

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
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.MultiModalBeliefBuilder;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;


/**
 * Dummy component for multi-modal estimation (currently directly converts a percept union
 * belief into a multi-modal belief)
 * 
 * @author Pierre Lison (plison@dfki.de)
 *
 */

public class MultiModalEstimation_fake extends FakeComponent {
 
		
	/**
	 * Adding change filters on working memory changes (insertion, overwrite, deletion)
	 * pertaining to percept union beliefs
	 * 
	 */
	@Override
	public void start() {
		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						unionBeliefAdded(_wmc);
					}
				}
		);
		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						unionBeliefUpdated(_wmc);
					}
				}
		);
		
		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						unionBeliefDeleted(_wmc);
					}
				}
		);
	}
	
	
	
	/**
	 * Upon the insertion of a new percept union belief, create one single offspring, add it to the
	 * belief offspring field, and update it
	 * 
	 * @param wmc the working memory change
	 */
	private void unionBeliefAdded (WorkingMemoryChange wmc) {
		
		try {
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, PerceptUnionBelief.class);
			
			addOffspring(beliefData.getData(), newDataID());	
			updateBeliefOnWM(beliefData.getData());
		}	

		 catch (Exception e) {
				e.printStackTrace();
			}
	}
	
	/**
	 * Upon the update of a given percept union, loop on its offspring.  If an offspring doesn't
	 * exist yet on the WM, create it, else update the existing one
	 * 
	 * @param wmc the working memory change
	 */
	private void unionBeliefUpdated (WorkingMemoryChange wmc) {
		
		try {
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, PerceptUnionBelief.class);

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
			for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					log("belief " + child.id + " exists on WM, overwriting");
					MultiModalBelief newChildBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), wmc.address, child.id);
				
					updatePointers(newChildBelief, MultiModalBelief.class);

					MultiModalBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), MultiModalBelief.class);
					newChildBelief.content = mergeBeliefContent(newChildBelief.content, existingBelief.content);

					updateBeliefOnWM(newChildBelief);
				}
				else {
					log("belief " + child.id + " does not exist on WM, creating it");
					MultiModalBelief childBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), wmc.address, child.id);
					updatePointers(childBelief, MultiModalBelief.class);
					insertBeliefInWM(childBelief);
				}
			}
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
	
	
	/**
	 * Upon the deletion of a given percept union, delete all the offspring of the belief
	 * 
	 * @param wmc the working memory change
	 */
	private void unionBeliefDeleted (WorkingMemoryChange wmc) {
		
		try {
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(wmc.address, PerceptUnionBelief.class);

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
