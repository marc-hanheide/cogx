package binder.abstr;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.utils.DistributionUtils;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import binder.ml.MLException;
import binder.utils.MLNGenerator;
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

public abstract class MarkovLogicComponent<T extends Belief> extends FakeComponent {

	/**
	 * 
	 */
	private T type;

	protected String markovlogicDir = "subarchitectures/binder/markovlogic/";

	private String inferCmd = "tools/alchemy/bin/infer";

	private String emptyFile = markovlogicDir + "empty.db";

	private String query = "Outcome";

	protected String MLNFile = markovlogicDir + "grouping.mln";
	protected String resultsFile = markovlogicDir + "unions.results";
	protected String predicatesFile = "";
	protected String correlationsFile = "";

	public float lowestProbThreshold = 0.08f;
	public int maxAlternatives = 2;
	public float minProbDifferenceForUpdate = 0.1f;

	protected String beliefUpdateToIgnore = "";

	
	public MarkovLogicComponent(T belief) {
		type = belief;
	}

	/**
	 * 
	 * @param mlnFile
	 * @param resultsFile
	 * @return
	 * @throws BeliefException
	 */
	public HashMap<String,Float> runAlchemyInference(String mlnFile, String resultsFile) throws BeliefException {

		Runtime run = Runtime.getRuntime(); 
		log("Now running Alchemy...");
		try {
			String[] args = {inferCmd, "-i", mlnFile, "-e", emptyFile, "-r", resultsFile, "-q", query};
			Process p = run.exec(args);

			BufferedReader stdInput = new BufferedReader(new InputStreamReader(p.getInputStream()));

			boolean inferenceSuccessful = true;

			String s;
			String output = "";
			while ((s = stdInput.readLine()) != null) {
				output += s + "\n";
				if (s.contains("ERROR")) {
					inferenceSuccessful = false;
				}
			}

			if (inferenceSuccessful) {
				log("Alchemy inference was successful, now retrieving the results");

				return readResultsFile (resultsFile);
			}

			else {
				log("ERROR: Alchemy inference failed");
				System.out.println(output);
				throw new BeliefException("ERROR: Alchemy inference failed");
			}
		}
		catch (IOException e) {
			e.printStackTrace();
		}
		return new HashMap<String,Float>();
	}
	
	protected abstract HashMap<String, Belief> extractExistingUnions();
	
	/**
	 * Perform the generic inference operation for the given belief, and subsequently update
	 * the working memory with the outcome (and possibly also with updates on existing ones)
	 * 
	 * @param belief the new belief which was inserted
	 */
	public void performInference(T belief, WorkingMemoryAddress beliefWMAddress) throws BeliefException {
		// extract the unions already existing in the binder WM
		Map<String, Belief> existingUnions = extractExistingUnions();
		
		Map<String, Belief> relevantUnions = selectRelevantUnions(existingUnions, belief);
		
		if (relevantUnions.size() > 0) {
			performInferenceSomethinToGroup(belief, beliefWMAddress,
					relevantUnions);
		}
		
		else {
			performInferenceNothingToGroup(belief, beliefWMAddress);
		}
	}

	private void performInferenceSomethinToGroup(T belief,
			WorkingMemoryAddress beliefWMAddress,
			Map<String, Belief> relevantUnions) {
		// Create identifiers for each possible new union		
		Map<String, String> unionsMapping = createIdentifiersForNewUnions(relevantUnions);
		
		String newSingleUnionId = newDataID();
		
		// unionsMapping.put("P", newSingleUnionId);
		// Write the Markov logic network to a file
		writeMarkovLogic(belief, relevantUnions, unionsMapping, newSingleUnionId);

		// run the alchemy inference
		try { 
			HashMap<String,Float> inferenceResults = runAlchemyInference(MLNFile, resultsFile);

			log("filtering inference results to keep only the " + maxAlternatives + " best alternatives");
			HashMap<String,Float> filteredInferenceResults = filterInferenceResults(inferenceResults);

			// create the new unions given the inference results
			Vector<Belief> newUnions = createNewUnions(belief, beliefWMAddress, relevantUnions,
					unionsMapping, newSingleUnionId, filteredInferenceResults);

			// and add them to the working memory
			performInferenceAddToWM(belief, newUnions);

			// modify the existence probabilities of the existing unions
			List<Belief> unionsToUpdate = getExistingUnionsToUpdate(belief, relevantUnions, unionsMapping, filteredInferenceResults);

			// and update them on the working memory
			performInferenceUpdateWM(unionsToUpdate);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void performInferenceUpdateWM(List<Belief> unionsToUpdate)
			throws DoesNotExistOnWMException, PermissionException,
			ConsistencyException {
		for (Belief unionToUpdate : unionsToUpdate) {
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

	private void performInferenceAddToWM(T belief, Vector<Belief> newUnions)
			throws AlreadyExistsOnWMException {
		for (Belief newUnion : newUnions) {
			if (DistributionUtils.getExistenceProbability(newUnion) > lowestProbThreshold)  {
				
				log("inserting belief " + newUnion.id + " on WM");

				updatePointersInCurrentBelief(newUnion);
				updatePointersInOtherBeliefs(newUnion);
					
				addOffspring(belief, newUnion.id);	
				beliefUpdateToIgnore = belief.id;

				try {
					insertBeliefInWM(newUnion);
				} catch (Exception e) {
					e.printStackTrace();
				} 
			}
			else {
				log ("Belief " + newUnion.id + " has probability " + DistributionUtils.getExistenceProbability(newUnion) +
				", which is lower than the minimum threshold.  Not inserting");
			}
		}
	}

	private void performInferenceNothingToGroup(T belief,
			WorkingMemoryAddress beliefWMAddress) {
		try {
			log("no relevant union to group with mmbelief " + belief.id + " has been found");
			Belief union = createNewSingleUnionBelief(belief, beliefWMAddress);
			if (DistributionUtils.getExistenceProbability(union) > lowestProbThreshold)  {
				
				log("inserting belief " + union.id + " on WM");

				updatePointersInCurrentBelief(union);
				updatePointersInOtherBeliefs(union);
					
				addOffspring(belief, union.id);	
				beliefUpdateToIgnore = belief.id;

				try {
					insertBeliefInWM(union);
				} catch (Exception e) {
					e.printStackTrace();
				} 
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	protected abstract Belief createNewSingleUnionBelief(T belief,
			WorkingMemoryAddress beliefWMAddress) throws BeliefException;

	private Map<String, String> createIdentifiersForNewUnions(
			Map<String, Belief> relevantUnions) {
		Map<String,String> unionsMapping = new HashMap<String,String>();
		
		for (String existingUnionId : relevantUnions.keySet()) {
			String newUnionId = newDataID();
			unionsMapping.put(newUnionId, existingUnionId);
		}
		
		log("newly created union ids: " + unionsMapping.keySet().toString());
		return unionsMapping;
	}

	private void writeMarkovLogic(T mmbelief,
			Map<String, Belief> relevantUnions,
			Map<String, String> unionsMapping, String newSingleUnionId) {
		try {
			MLNPreferences prefs = new MLNPreferences();
			prefs.setFile_correlations(correlationsFile);
			prefs.setFile_predicates(predicatesFile);
			MLNGenerator gen = new MLNGenerator(prefs);
			gen.writeMLNFile(mmbelief, relevantUnions.values(), unionsMapping, newSingleUnionId, MLNFile);
		} catch (MLException e1) {
			e1.printStackTrace();
		}
	}

	/**
	 * 
	 * @param resultsFile
	 * @return
	 */
	public HashMap<String,Float> readResultsFile (String resultsFile) {
		HashMap<String,Float> inferenceResults = new HashMap<String,Float>();

		try {
			FileInputStream fstream = new FileInputStream(resultsFile);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			while ((strLine = br.readLine()) != null)   {
				log (strLine);
				String line2 = strLine.replace(query+"(", "");
				String markovlogicunion = line2.substring(0, line2.indexOf(")"));
				float prob = Float.parseFloat(line2.substring(line2.indexOf(" ")));

				String union = MLNGenerator.getIDFromMarkovLogicConstant(markovlogicunion);

				inferenceResults.put(union, prob);
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		return inferenceResults;
	}

	/**
	 * 
	 * @param perceptWMAddress
	 */
	public abstract void workingMemoryChangeDelete(WorkingMemoryAddress perceptWMAddress);

	/**
	 * Filter the inference results to retain only the maxSize-best results
	 * 
	 * @param results the results of Alchemy inference
	 * @return the filtered results
	 */
	protected HashMap<String, Float> filterInferenceResults(HashMap<String,Float> results) {

		List<Entry<String,Float>> list = new LinkedList<Entry<String,Float>>(results.entrySet());
		Collections.sort(list, new Comparator<Entry<String,Float>>() {
			public int compare(Entry<String,Float> o1, Entry<String,Float> o2) {
				return -(Float.compare(o1.getValue().floatValue(), o2.getValue().floatValue()));
			}
		});

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
	 * 
	 * @param b
	 * @return
	 * @throws BeliefException
	 */
	protected List<String> getOriginSubarchitectures(Belief b) throws BeliefException {

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

	protected void deleteAllMultiModalBeliefAttachedToUnion(WorkingMemoryAddress unionAddress) {
	
		try {
			CASTData<MultiModalBelief>[] mmBeliefs;
	
			mmBeliefs = getWorkingMemoryEntries(BindingWorkingMemory.BINDER_SA, MultiModalBelief.class);
	
			for (int i = (mmBeliefs.length - 1) ; i >= 0 ; i--) {
				MultiModalBelief mmbelief = mmBeliefs[i].getData();
				if (mmbelief != null && mmbelief.hist != null && mmbelief.hist instanceof CASTBeliefHistory) {
					if (((CASTBeliefHistory)mmbelief.hist).ancestors.contains(unionAddress)) {
						deleteBeliefOnWM (mmBeliefs[i].getID());
					}
				}
			}
		}
		catch (UnknownSubarchitectureException e) {
			log("Problem with architecture name!");
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
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
	protected List<Belief> getExistingUnionsToUpdate(
			T percept,
			Map<String,Belief> existingUnions,
			Map<String,String> linkToExistingUnions,
			Map<String,Float> inferenceResults) {

		List<Belief> unionsToUpdate = new LinkedList<Belief>();

		// extract the existence probability of the percept
		float perceptExistProb = DistributionUtils.getExistenceProbability(percept);

		for (String newUnionId: linkToExistingUnions.keySet()) {
			Belief existingUnion = existingUnions.get(linkToExistingUnions.get(newUnionId));
			float unionCurrentExistProb = DistributionUtils.getExistenceProbability(existingUnion);
			float unionNewExistProb = (unionCurrentExistProb * (1-perceptExistProb)) + 
			(perceptExistProb * (1 - inferenceResults.get(newUnionId)) * unionCurrentExistProb);
			log("new prob for " +  linkToExistingUnions.get(newUnionId) + ": " + unionNewExistProb);

			try {
				DistributionUtils.setExistenceProbability(existingUnion, unionNewExistProb);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			if (Math.abs(unionNewExistProb - unionCurrentExistProb) > minProbDifferenceForUpdate) {
				unionsToUpdate.add(existingUnion);
				log("Existing union " + existingUnion.id + " going from existence probability " + unionCurrentExistProb + 
						" to probabibility " + unionNewExistProb + ", update necessary");
			}
		}
		return unionsToUpdate;
	}

	/**
	 * Only selecting the unions which are originating from the same subarchitecture as the current 
	 * mmbelief
	 * 
	 * @param existingUnions
	 * @param belief
	 * @return
	 * @throws BeliefException
	 */
	protected Map<String,Belief> selectRelevantUnions(Map<String, Belief> existingUnions, T belief) throws BeliefException {
		
		Map<String,Belief> relevantUnions = new HashMap<String,Belief>();
		
		if (((CASTBeliefHistory)belief.hist).ancestors.size() == 0) {
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
		}
		return relevantUnions;
	}
	
	protected abstract Vector<Belief> createNewUnions(
			T percept,
			WorkingMemoryAddress perceptWMAddress,
			Map<String,Belief> existingUnions,
			Map<String,String> unionsMapping,
			String newSingleUnionId,
			Map<String,Float> inferenceResults) throws BeliefException;
	
	protected abstract void updatePointersInOtherBeliefs (Belief newBelief);


}
