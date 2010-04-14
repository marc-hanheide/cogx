package binder.components;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.PerceptUnionBuilder;
import beliefmodels.utils.DistributionUtils;
import binder.abstr.MarkovLogicComponent;
import binder.arch.BindingWorkingMemory;
import binder.ml.MLException;
import binder.utils.MLNGenerator;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Perceptual grouping operation, responsible for merging percept beliefs from different modalities into
 * percept union beliefs. 
 * 
 * The component continuously monitors changes on the binder working memory, and triggers its internal 
 * inference mechanism (based on a Markov Logic Network) when a percept is being inserted, updated or
 * deleted. The final outcome of this inference is the creation of new percept union beliefs which are
 * then inserted onto the working memory, as well as the associated update of the existing percept union
 * beliefs.
 * 
 * 
 * 
 * NOTE: 
 * - still need to check subarchitecture consistency OK
 * - still need to add filters OK
 * - only perform updates on existing unions when change is significant OK
 * - need to add functionality for percept updates or deletions
 * - remove the testing stuff and have a proper, separate tester class  OK
 * - actually build the union content OK
 * - change the belief history to have only cast values OK
 * - testing, defensive programming, check null values and pre/post conditions
 * - when constructing a new belief, propagate the pointers correctly
 * - have proper logging functionality
 * - implement the same kind of functionality for tracking
 * - send examples of test cases to Sergio
 * - do the test with percepts instead of unions as inputs
 * 
 * @author plison
 *
 */
public class PerceptualGrouping_MLN extends MarkovLogicComponent {

	
	String MLNFile = markovlogicDir + "grouping.mln";
	
	String resultsFile = markovlogicDir + "unions.results";
	
	// lowest probability threshold for the existence probability of a percept union
	public float lowestProbThreshold = 0.08f;
	
	// maximum number of alternatives to consider for perceptual grouping.  If more are available, take the highest-probability ones
	public int maxAlternatives = 2;
	
	// minimum difference in probability in order to trigger a working memory update on an existing union
	public float minProbDifferenceForUpdate = 0.1f;
	
	
	/**
	 * Add a change filter on the insertion of new percept beliefs on the binder working memory
	 */
	@Override
	public void start() {
				
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);	
							
							log("received a new percept: " + beliefData.getID());
							performPerceptualGrouping (beliefData.getData(), _wmc.address);
							log("perceptual grouping operation on percept " + beliefData.getID() + " now finished");
						}	
			
						 catch (Exception e) {
								e.printStackTrace();
							} 
					}
				}
		);
	}
  
	
	
	/**
	 * Perform the perceptual grouping operation for the given belief, and subsequently update
	 * the working memory with new percept unions (and possibly also with updates on existing ones)
	 * 
	 * @param percept the new percept which was inserted
	 */
	public void performPerceptualGrouping(PerceptBelief percept, WorkingMemoryAddress perceptWMAddress) throws BeliefException {
	
		log("now starting perceptual grouping...");

		// extract the unions already existing in the binder WM
		HashMap<String, PerceptUnionBelief> existingUnions = extractExistingUnions();
		
		HashMap<String, PerceptUnionBelief> relevantUnions = selectRelevantUnions(existingUnions, percept);
		
		if (relevantUnions.size() > 0) {
		// Create identifiers for each possible new union		
		HashMap<String,String> unionsMapping = new HashMap<String,String>();
		for (String existingUnionId : relevantUnions.keySet()) {
			String newUnionId = newDataID();
			unionsMapping.put(newUnionId, existingUnionId);
		}
		log("newly created union ids: " + unionsMapping.keySet().toString());
			
		String newSingleUnionId = newDataID();
	//	unionsMapping.put("P", newSingleUnionId);
		
		// Write the markov logic network to a file
		try {
			(new MLNGenerator()).writeMLNFile(percept, relevantUnions.values(), unionsMapping, newSingleUnionId, MLNFile);
		} catch (MLException e1) {
			e1.printStackTrace();
		}
		
		// run the alchemy inference
		try { 
		HashMap<String,Float> inferenceResults = runAlchemyInference(MLNFile, resultsFile);
		
		log("filtering inference results to keep only the " + maxAlternatives + " best alternatives");
		HashMap<String,Float> filteredInferenceResults = filterInferenceResults(inferenceResults, maxAlternatives);
		
		// create the new unions given the inference results
		Vector<PerceptUnionBelief> newUnions = createNewUnions(percept, perceptWMAddress, relevantUnions,
				unionsMapping, newSingleUnionId, filteredInferenceResults);

		// and add them to the working memory
		for (PerceptUnionBelief newUnion : newUnions) {
			if (DistributionUtils.getExistenceProbability(newUnion) > lowestProbThreshold)  {
				insertBeliefInWM(newUnion);
				log("inserting belief " + newUnion.id + " on WM");
			}
			else {
				log ("Belief " + newUnion.id + " has probability " + DistributionUtils.getExistenceProbability(newUnion) +
						", which is lower than the minimum threshold.  Not inserting");
			}
		}

		// modify the existence probabilities of the existing unions
		List<PerceptUnionBelief> unionsToUpdate = getExistingUnionsToUpdate(percept, relevantUnions, unionsMapping, filteredInferenceResults);
		
		// and update them on the working memory
		for (PerceptUnionBelief unionToUpdate : unionsToUpdate) {
			if (DistributionUtils.getExistenceProbability(unionToUpdate) > lowestProbThreshold)  {
				log("updating belief " + unionToUpdate.id + " on WM");
				updateBeliefOnWM(unionToUpdate);
			}
			else  {
				log("deleting belief: " + unionToUpdate.id + " on WM");
				deleteBeliefOnWM(unionToUpdate.id);
			}
		}
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		}
		else {
			log("no relevant union to group with percept " + percept.id + " has been found");
		}
	}


	private HashMap<String, Float> filterInferenceResults (HashMap<String,Float> results, int maxSize) {
		
		     List<Entry<String,Float>> list = new LinkedList<Entry<String,Float>>(results.entrySet());
		     Collections.sort(list, new Comparator<Entry<String,Float>>() {
		          public int compare(Entry<String,Float> o1, Entry<String,Float> o2) {
		               if (o1.getValue().floatValue() >= o2.getValue().floatValue()) {
		            	   return -1;
		               }
		              return 1;
		          }
		     });
		// logger.info(list);
		
		int increment = 0;
		HashMap<String,Float> newResults = new HashMap<String,Float>();
		for (Iterator<Entry<String,Float>> it = list.iterator(); it.hasNext();) {
		     Map.Entry<String,Float> entry = (Map.Entry<String,Float>)it.next();
		     if (increment < maxAlternatives) {
		     newResults.put(entry.getKey(), entry.getValue());
		     increment++;
		     }
		     else {
		    	 newResults.put(entry.getKey(), new Float(0.0f)); 
		     }
		     }
		return newResults;
	}
	
	
	/**
	 * Create a set of new percept union beliefs from the inference results, associated with the 
	 * original percept
	 * @param percept
	 * @param linkToExistingUnions
	 * @param inferenceResults
	 * @return
	 */
	private Vector<PerceptUnionBelief> createNewUnions(
			PerceptBelief percept,
			WorkingMemoryAddress perceptWMAddress,
			HashMap<String,PerceptUnionBelief> existingUnions,
			HashMap<String,String> unionsMapping,
			String newSingleUnionId,
			HashMap<String,Float> inferenceResults) throws BeliefException {

		// extract the existence probability of the percept
		float perceptExistProb = DistributionUtils.getExistenceProbability(percept);

		Vector<PerceptUnionBelief> newUnions = new Vector<PerceptUnionBelief>();
		for (String id : unionsMapping.keySet()) {
			
			if (!inferenceResults.containsKey(id)) {
				throw new BeliefException("ERROR, id " + id + " is not in inferenceResults.  inferenceResults = " + inferenceResults.keySet().toString());
			}
			
			else if (!existingUnions.containsKey(unionsMapping.get(id))) {
				throw new BeliefException("ERROR, existing union id " + unionsMapping.get(id) + " is not in existingUnions");
			}
			
			float prob = perceptExistProb * inferenceResults.get(id);
			log("computed prob of " + id + ": " + prob);
			
			PerceptUnionBelief existingUnion = existingUnions.get(unionsMapping.get(id)); 
			try {
			PerceptUnionBelief newUnion = PerceptUnionBuilder.createNewDoubleUnionBelief(percept, perceptWMAddress, existingUnion, prob, id);
			newUnions.add(newUnion);
			}
			catch (BeliefException e) {
				e.printStackTrace();
			}
		} 
		
		if (!inferenceResults.containsKey(newSingleUnionId)) {
			throw new BeliefException("ERROR, id " + newSingleUnionId + " is not in inferenceResults");
		}
		PerceptUnionBelief newSingleUnion = 
			PerceptUnionBuilder.createNewSingleUnionBelief(percept, perceptWMAddress, inferenceResults.get(newSingleUnionId), newSingleUnionId);
		
		newUnions.add(newSingleUnion);
		
		return newUnions;
	}
	 
	 
	/**
	 * Modify the existence probability of existing unions, based on the inference results, 
	 * the newly inserted percept, the set of existing unions, and the mapping between the new 
	 * and the already existing unions
	 * 
	 * @param percept the percept
	 * @param existingUnions the set of existing unions
	 * @param linkToExistingUnions mapping from new unions to existing unions they include
	 * @param inferenceResults the inference results
	 */
	private List<PerceptUnionBelief> getExistingUnionsToUpdate (
			PerceptBelief percept, 
			HashMap<String,PerceptUnionBelief> existingUnions, 
			HashMap<String,String> linkToExistingUnions,
			HashMap<String,Float> inferenceResults) {
		
		List<PerceptUnionBelief> unionsToUpdate = new LinkedList<PerceptUnionBelief>();
		
		// extract the existence probability of the percept
		float perceptExistProb = DistributionUtils.getExistenceProbability(percept);
		
		for (String newUnionId: linkToExistingUnions.keySet()) {
			PerceptUnionBelief existingUnion = existingUnions.get(linkToExistingUnions.get(newUnionId));
			float unionCurrentExistProb = DistributionUtils.getExistenceProbability(existingUnion);
			float unionNewExistProb = (unionCurrentExistProb * (1-perceptExistProb)) + 
				(perceptExistProb * (1 - inferenceResults.get(newUnionId)) * unionCurrentExistProb);
			log("new prob for " +  linkToExistingUnions.get(newUnionId) + ": " + unionNewExistProb);
			
			DistributionUtils.setExistenceProbability(existingUnion, unionNewExistProb);
			
			if (Math.abs(unionNewExistProb - unionCurrentExistProb) > minProbDifferenceForUpdate) {
				unionsToUpdate.add(existingUnion);
				log("Existing union " + existingUnion.id + " going from existence probability " + unionCurrentExistProb + 
						" to probabibility " + unionNewExistProb + ", update necessary");
			}
		}
		return unionsToUpdate;
	}  
	 
	
	private List<String> getOriginSubarchitectures(Belief b) throws BeliefException {
		
		List<String> subarchitectures = new ArrayList<String>();
		
		if (!(b.hist instanceof CASTBeliefHistory)) {
			throw new BeliefException ("ERROR: history not specified as cast pointer");
		}
		else if (((CASTBeliefHistory)b.hist).ancestors == null) {
			throw new BeliefException ("ERROR: percept history is null");
		}
	
		for (WorkingMemoryAddress ancestorAddress : ((CASTBeliefHistory)b.hist).ancestors) {
			if (!(ancestorAddress.subarchitecture.equals(BindingWorkingMemory.BINDER_SA))) {
				subarchitectures.add(ancestorAddress.subarchitecture);
			} else
				try {
					if (existsOnWorkingMemory(ancestorAddress)) {
						Belief subB = getMemoryEntry(ancestorAddress, Belief.class);
						subarchitectures.addAll(getOriginSubarchitectures(subB));
					}
				} catch (Exception e) {
					e.printStackTrace();
				}
		}
		return subarchitectures;
	}
	
	
	private HashMap<String,PerceptUnionBelief> selectRelevantUnions (HashMap<String,PerceptUnionBelief> existingUnions, 
			PerceptBelief percept) throws BeliefException {
		
		HashMap<String,PerceptUnionBelief> relevantUnions = new HashMap<String,PerceptUnionBelief>();
	
		if (((CASTBeliefHistory)percept.hist).ancestors.size() == 0) {
			throw new BeliefException ("ERROR: percept history contains 0 element");
		}
		else if (((CASTBeliefHistory)percept.hist).ancestors.size() > 1) {
			throw new BeliefException ("ERROR: percept history contains more than 1 element");
		}
		
		String perceptOrigin = getOriginSubarchitectures(percept).get(0);
		
		for (String existingUnionId : existingUnions.keySet()) {
			
			PerceptUnionBelief existingUnion = existingUnions.get(existingUnionId);
			List<String> existinUnionOrigins = getOriginSubarchitectures(existingUnion);
			
			if (!existinUnionOrigins.contains(perceptOrigin)) {
				relevantUnions.put(existingUnionId, existingUnion);
			}
			
		}
		
		
		return relevantUnions;
	}
	
	/**
	 * Exact the set of existing unions from the binder working memory
	 * 
	 * @return the set of existing unions (as a mapping from identifier to objects)
	 */
	private HashMap<String, PerceptUnionBelief> extractExistingUnions() {

		HashMap<String, PerceptUnionBelief> existingunions = new HashMap<String, PerceptUnionBelief>();

		try {
			CASTData<PerceptUnionBelief>[] unions;

			unions = getWorkingMemoryEntries(BindingWorkingMemory.BINDER_SA, PerceptUnionBelief.class);

			for (int i = (unions.length - 1) ; i >= 0 ; i--) {
				existingunions.put(unions[i].getData().id, unions[i].getData());
			}
		}
		catch (UnknownSubarchitectureException e) {
			log("Problem with architecture name!");
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
		return existingunions;
	}
	
	
	
	private void deleteAllMultiModalBeliefAttachedToUnion (PerceptUnionBelief union) {
		

//		HashMap<String, PerceptUnionBelief> existingunions = new HashMap<String, PerceptUnionBelief>();

		try {
			CASTData<MultiModalBelief>[] mmBeliefs;

			mmBeliefs = getWorkingMemoryEntries(BindingWorkingMemory.BINDER_SA, MultiModalBelief.class);

			for (int i = (mmBeliefs.length - 1) ; i >= 0 ; i--) {
				// TODO!!!
			}
		}
		catch (UnknownSubarchitectureException e) {
			log("Problem with architecture name!");
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}
	
	
}
