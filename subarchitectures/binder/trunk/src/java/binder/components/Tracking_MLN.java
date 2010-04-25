package binder.components;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.TemporalUnionBuilder;
import beliefmodels.utils.DistributionUtils;
import beliefmodels.utils.FeatureContentUtils;
import binder.abstr.MarkovLogicComponent;
import binder.arch.BindingWorkingMemory;
import binder.utils.MLNPreferences;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * Perceptual grouping operation, responsible for merging percept beliefs from
 * different modalities into percept union beliefs.
 * 
 * The component continuously monitors changes on the binder working memory, and
 * triggers its internal inference mechanism (based on a Markov Logic Network)
 * when a percept is being inserted, updated or deleted. The final outcome of
 * this inference is the creation of new percept union beliefs which are then
 * inserted onto the working memory, as well as the associated update of the
 * existing percept union beliefs.
 * 
 * 
 * 
 * 
 * @author plison
 * 
 */
public class Tracking_MLN extends MarkovLogicComponent<MultiModalBelief> {


	String generatedMLNFile = markovlogicDir + "tracking.mln";
	HashMap<String,String> trackerMLNs = new HashMap<String,String>();
	
	Vector<MultiModalBelief> beliefUpdatesToIgnore = new Vector<MultiModalBelief>();

	public Tracking_MLN() {
		super(MultiModalBelief.class);
		resultsFile = markovlogicDir + "tracking.results";
		beliefUpdatesToIgnore = new Vector<MultiModalBelief>();
		trackerMLNs.put("Object", MLNPreferences.markovlogicDir + "tracking/tracking-objects.mln");
		trackerMLNs.put("Person", MLNPreferences.markovlogicDir + "tracking/tracking-persons.mln");
	}

	

	/**
	 * Add a change filter on the insertion of new percept beliefs on the binder working memory
	 */
	public void start() {
		
		Runtime run = Runtime.getRuntime(); 
		log("Verifying that Alchemy is correctly compiled...");
		String[] args = {inferCmd};
		try {
			Process p = run.exec(args);
		} catch (IOException e1) {
			System.out.println("FATAL ERROR: tools/alchemy/bin/infer is not found.  Alchemy package does not seem to be properly compiled.  Exiting...");
			System.exit(0);
		}

		
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);

							log("received a new belief: " + beliefData.getID());
	
							inference(beliefData.getData(), _wmc.address);
							
							beliefUpdatesToIgnore.add(beliefData.getData());
							updateBeliefOnWM(beliefData.getData());

						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
		);

		
		// Deletion

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);

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
		);

		
		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					@SuppressWarnings("unchecked")
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							CASTData<MultiModalBelief> beliefData = 
								getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);
							
							if (!beliefUpdatesToIgnore.contains(beliefData.getData())) {
								
								List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
								
								for (WorkingMemoryAddress child : offspring) {
									if (existsOnWorkingMemory(child)) {
										log("belief " + child.id + " exists on WM, overwriting");
										TemporalUnionBelief childBelief = getMemoryEntry(child, TemporalUnionBelief.class);
										childBelief = TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, child.id);
										updatePointers(childBelief, TemporalUnionBelief.class);
										updateBeliefOnWM(childBelief);
									}
									else {
										log("belief " + child.id + " does not exist on WM, creating it");
										TemporalUnionBelief childBelief = TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, child.id);
										updatePointers(childBelief, TemporalUnionBelief.class);
										insertBeliefInWM(childBelief);
									}
								}
								
							}
							else {
								log("ignoring overwrite update (offspring change)");
								beliefUpdatesToIgnore.remove(beliefData.getData());
							}
 							
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
		);
	}

	
	private void inference (MultiModalBelief belief, WorkingMemoryAddress beliefWMAddress)
		throws BeliefException, DoesNotExistOnWMException, PermissionException, ConsistencyException, 
		AlreadyExistsOnWMException, UnknownSubarchitectureException {
	
		
		MultiModalBelief beliefCopy = duplicateBelief(belief);
		updatePointers(beliefCopy, TemporalUnionBelief.class);
		List<Belief> results = performInference(beliefCopy, beliefWMAddress, getPreferences(belief));

		for (Belief b : results) {
			if (existsOnWorkingMemory(new WorkingMemoryAddress(b.id, BindingWorkingMemory.BINDER_SA))) {
				log("belief " + b.id + " exists on WM, overwriting");
				updateBeliefOnWM(b);
			}
			else {
				log("belief " + b.id + " does not exist on WM, creating it");
				insertBeliefInWM(b);
			}
			addOffspring(belief, b.id);	
		}
	}
	
	
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
	 * @return the set of existing unions (as a mapping from identifier to
	 *         objects)
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

	@Override
	protected Belief createNewSingleUnionBelief(MultiModalBelief belief,
			WorkingMemoryAddress beliefWMAddress) throws BeliefException {
		TemporalUnionBelief union = TemporalUnionBuilder.createNewSingleUnionBelief(belief, beliefWMAddress, newDataID());
		return union;
	}
	
	
	protected Map<String,Belief> selectRelevantUnions(Map<String, Belief> existingUnions, MultiModalBelief belief) throws BeliefException {
		
		Map<String,Belief> relevantUnions = new HashMap<String,Belief>();
		
		for (String existingUnionId: existingUnions.keySet()) {
			Belief existingUnion = existingUnions.get(existingUnionId);
			if (existingUnion.type.equals(belief.type)) {
				relevantUnions.put(existingUnionId, existingUnion);
			}
		}
		
		/**	if (((CASTBeliefHistory)belief.hist).ancestors.size() == 0) {
			throw new BeliefException ("ERROR: belief history contains 0 element");
		}
		
		for(String mmbeliefOrigin : getOriginSubarchitectures(belief)) {
			for(String existingUnionId : existingUnions.keySet()) {
				
				Belief existingUnion = existingUnions.get(existingUnionId);
				List<String> existinUnionOrigins = getOriginSubarchitectures(existingUnion);
				
				if (existinUnionOrigins.contains(mmbeliefOrigin)) {
					relevantUnions.put(existingUnionId, existingUnion);
				}
			}
		} */
		return relevantUnions;
	}

}
