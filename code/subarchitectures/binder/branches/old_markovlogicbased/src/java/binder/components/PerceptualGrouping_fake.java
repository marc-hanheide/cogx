

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

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.PerceptUnionBuilder;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Dummy component for perceptual grouping (currently directly converts a percept
 * belief into a percept union belief)
 * 
 * @author Pierre Lison (plison@dfki.de)
 *
 */
public class PerceptualGrouping_fake extends FakeComponent {

	/**
	 * Adding change filters on working memory changes (insertion, overwrite, deletion)
	 * pertaining to percept beliefs
	 * 
	 */
	@Override
	public void start() {
		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefAdded(_wmc);
					}
				}
		);
		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefUpdated(_wmc);
					}
				}
		);
		
		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefDeleted(_wmc);
					}
				}
		);
	}

	
	/**
	 * Upon the insertion of a new percept, create one single offspring, add it to the
	 * belief offspring field, and update it
	 * 
	 * @param wmc the working memory change
	 */
	private void perceptBeliefAdded (WorkingMemoryChange wmc) {
		try {
			
			// extract the insert belief
			CASTData<PerceptBelief> beliefData = 
				getMemoryEntryWithData(wmc.address, PerceptBelief.class);

			addOffspring(beliefData.getData(), newDataID());
			updateBeliefOnWM(beliefData.getData());
			
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
	
	/**
	 * Upon the update of a given percept, loop on its offspring.  If an offspring doesn't
	 * exist yet on the WM, create it, else update the existing one
	 * 
	 * @param wmc the working memory change
	 */
	private void perceptBeliefUpdated (WorkingMemoryChange wmc) {
		
		try {
			CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(wmc.address, PerceptBelief.class);

			log("overwriting belief " + beliefData.getID());

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;

			log("number of offspring for belief " + beliefData.getID() + ": " + offspring.size());

			for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					log("belief " + child.id + " exists on WM, overwriting");
					PerceptUnionBelief newChildBelief = 
						PerceptUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), wmc.address, child.id);
				
					updatePointers(newChildBelief, PerceptUnionBelief.class);

					PerceptUnionBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), PerceptUnionBelief.class);
					newChildBelief.content = mergeBeliefContent(existingBelief.content, newChildBelief.content);
					
					updateBeliefOnWM(newChildBelief);
				}
				else {
					log("belief " + child.id + " does not exist on WM, creating it");
					PerceptUnionBelief childBelief = PerceptUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), wmc.address, child.id);
					
					updatePointers(childBelief, PerceptUnionBelief.class);
					insertBeliefInWM(childBelief);
				}
			}
			
			if (offspring.size() == 0) {
				log("WARNING: " + beliefData.getID() + " has no offspring, something wrong here");
			}
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
	
	
	/**
	 * Upon the deletion of a given percept, delete all the offspring of the belief
	 * 
	 * @param wmc the working memory change
	 */
	private void perceptBeliefDeleted (WorkingMemoryChange wmc) {
		
		try {
			PerceptBelief beliefData = getMemoryEntry(wmc.address, PerceptBelief.class);

			List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.hist).offspring;
			
		/**	for (WorkingMemoryAddress child : offspring) {
				if (existsOnWorkingMemory(child)) {
					deleteBeliefOnWM(child.id);
				}
			} */
			
		}	

		catch (Exception e) {
			e.printStackTrace();
		} 
	}
}
