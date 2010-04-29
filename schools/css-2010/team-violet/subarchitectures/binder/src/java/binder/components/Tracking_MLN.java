
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

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.TemporalUnionBuilder;
import binder.abstr.MarkovLogicComponent;
import binder.arch.BindingWorkingMemory;
import binder.utils.MLNPreferences;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * Tracking operation, responsible for merging multi-modal beliefs observed at different
 * time points into temporal union beliefs.
 * 
 * The component continuously monitors changes on the binder working memory, and
 * triggers its internal inference mechanism (based on a Markov Logic Network)
 * when a multi-modal belief is being inserted, updated or deleted. The final outcome of
 * this inference is the creation of new temporal union beliefs which are then
 * inserted onto the working memory, as well as the associated update of the
 * existing temporal union beliefs.
 * 
 * 
 * @author Pierre Lison & Carsten Ehrler (plison@dfki.de)
 * @version April 25, 2010
 * 
 */
public class Tracking_MLN extends MarkovLogicComponent<MultiModalBelief> {


	// the full markov logic file which is going to be generated
	String generatedMLNFile = markovlogicDir + "tracking.mln";
	
	// the list of possible tracking systems
	HashMap<String,String> trackerMLNs = new HashMap<String,String>();
	
	// the set of beliefs whose overwrite can be temporarily ignored (because the update
	// only concerns the offspring property)
	private Vector<MultiModalBelief> beliefUpdatesToIgnore = new Vector<MultiModalBelief>();

	
	/**
	 * Starting the MLN-based tracker
	 */
	public Tracking_MLN() {
		super();
		resultsFile = markovlogicDir + "tracking.results";
		beliefUpdatesToIgnore = new Vector<MultiModalBelief>();
		trackerMLNs.put("Object", MLNPreferences.markovlogicDir + "tracking/tracking-objects.mln");
		trackerMLNs.put("object", MLNPreferences.markovlogicDir + "tracking/tracking-objects.mln");
		trackerMLNs.put("Person", MLNPreferences.markovlogicDir + "tracking/tracking-persons.mln");
		trackerMLNs.put("person", MLNPreferences.markovlogicDir + "tracking/tracking-persons.mln");
	}

	

	/**
	 * Add change filters on the insertion of new multi-modal beliefs on the binder working memory
	 */
	public void start() {
		
		// verifying that Alchemy has been properly compiled
		verifyAlchemyIsPresent();

		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						multimodalBeliefAdded(_wmc);
					}
				}
		);

		
		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						multimodalBeliefDeleted(_wmc);
					}
				}
		);

		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						multimodalBeliefOverwritten(_wmc);
					}
				}
		);
	}

	
	/**
	 * Verify whether the Alchemy software is present in the system
	 */
	private void verifyAlchemyIsPresent () {
		Runtime run = Runtime.getRuntime(); 
		log("Verifying that Alchemy is correctly compiled...");
		String[] args = {inferCmd};
		try {
			Process p = run.exec(args);
		} catch (IOException e1) {
			System.out.println("FATAL ERROR: tools/alchemy/bin/infer is not found.  " + 
					"Alchemy package does not seem to be properly compiled.  Exiting...");
			System.exit(0);
		}
	}
	
	
	/**
	 * Perform inference if new multi-modal belief added in the system
	 * 
	 * @param wmc working memory change
	 */
	private void multimodalBeliefAdded (WorkingMemoryChange wmc) {
		try {
			CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(wmc.address, MultiModalBelief.class);

			log("received a new belief: " + beliefData.getID());

			performInference(beliefData.getData(), wmc.address);
			
			beliefUpdatesToIgnore.add(beliefData.getData());
			updateBeliefOnWM(beliefData.getData());

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Delete the offspring of a given multi-modal belief
	 * 
	 * @param wmc working memory change
	 */
	private void multimodalBeliefDeleted (WorkingMemoryChange wmc) {

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
	
	
	/**
	 * Perform an inference if a given multi-modal belief is overwritten (updated) in the
	 * working memory
	 * 
	 * @param wmc working memory change
	 */
	private void multimodalBeliefOverwritten (WorkingMemoryChange wmc) {
		
		try {
			CASTData<MultiModalBelief> beliefData = 
				getMemoryEntryWithData(wmc.address, MultiModalBelief.class);
			
			// only perform update if the belief in not in the "ignore" list
			if (!beliefUpdatesToIgnore.contains(beliefData.getData())) {
				
				List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
				
				// looping on the list of offspring
				for (WorkingMemoryAddress child : offspring) {
					if (existsOnWorkingMemory(child)) {
						log("belief " + child.id + " exists on WM, overwriting");
						TemporalUnionBelief childBelief = TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), wmc.address, child.id);
					
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
			else {
			//	log("ignoring overwrite update (offspring change)");
				beliefUpdatesToIgnore.remove(beliefData.getData());
			}
				
		}
		catch (Exception e) {
			e.printStackTrace();
		}

	}
	

	/**
	 * Perform inference on the newly added multi-modal belief (associated with its address).  
	 * The belief pointers are first updated, then Markov Logic inference is triggered, and the results
	 * are inserted into the working memory
	 * 
	 * @param belief the multi-modal belief
	 * @param beliefWMAddress the address of the belief in the WM
	 */
	
	private void performInference (MultiModalBelief belief, WorkingMemoryAddress beliefWMAddress)  {
	
		try {
		// duplicate the current multi-modal belief, and update the pointers
		MultiModalBelief beliefCopy = duplicateBelief(belief);
		updatePointers(beliefCopy, TemporalUnionBelief.class);
		
		// perform the inference itself
		List<Belief> results = performInference(beliefCopy, beliefWMAddress, getPreferences(belief));

		// update an existing belief or insert a new one
		for (Belief b : results) {
			if (existsOnWorkingMemory(new WorkingMemoryAddress(b.id, BindingWorkingMemory.BINDER_SA))) {
				log("belief " + b.id + " exists on WM, overwriting");
				
				TemporalUnionBelief childBelief = 
					TemporalUnionBuilder.createNewSingleUnionBelief(belief, beliefWMAddress, b.id);
			
				updatePointers(childBelief, TemporalUnionBelief.class);

				TemporalUnionBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(b.id, BindingWorkingMemory.BINDER_SA), TemporalUnionBelief.class);
				childBelief.content = mergeBeliefContent(existingBelief.content, childBelief.content);

				updateBeliefOnWM(childBelief);

			}
			else {
				log("belief " + b.id + " does not exist on WM, creating it");
				insertBeliefInWM(b);
			}
			addOffspring(belief, b.id);	
		}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	
	/**
	 * Get the preferences to set (tracker system etc.) for a particular belief
	 * 
	 * @param b the belief on which the inference is to be made
	 * @return the preferences
	 */
	private MLNPreferences getPreferences(Belief b) {
		MLNPreferences prefs = new MLNPreferences();
		
		prefs.setGeneratedMLNFile(generatedMLNFile);
		
		for (String key : trackerMLNs.keySet()) {
			if (b.type.contains(key)) {
				prefs.setFile_correlations(trackerMLNs.get(key));
				prefs.activateTracking();
			}
		}

		return prefs;
	}

	

	/**
	 * Exact the set of existing unions from the binder working memory
	 * 
	 * @return the set of existing unions (as a mapping from identifier to objects)
	 */
	protected HashMap<String, Belief> extractExistingUnions() {

		HashMap<String, Belief> existingunions = new HashMap<String, Belief>();

		try {
			CASTData<TemporalUnionBelief>[] unions;

			unions = getWorkingMemoryEntries(BindingWorkingMemory.BINDER_SA,
					TemporalUnionBelief.class);

			for (int i = (unions.length - 1); i >= 0; i--) {
				existingunions.put(unions[i].getData().id, unions[i].getData());
			}
		} catch (UnknownSubarchitectureException e) {
			log("Problem with architecture name!");
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
		return existingunions;
	}

	
	/**
	 * Select the relevant set of unions on which to track the new belief, based on the
	 * belief types (which must be identical).
	 * 
	 */
	protected Map<String,Belief> selectRelevantUnions(Map<String, Belief> existingUnions, 
			MultiModalBelief belief) throws BeliefException {
		
		Map<String,Belief> relevantUnions = new HashMap<String,Belief>();
		
		for (String existingUnionId: existingUnions.keySet()) {
			Belief existingUnion = existingUnions.get(existingUnionId);
			if (existingUnion.type.equals(belief.type)) {
				relevantUnions.put(existingUnionId, existingUnion);
			}
		}
		return relevantUnions;
	}

}
