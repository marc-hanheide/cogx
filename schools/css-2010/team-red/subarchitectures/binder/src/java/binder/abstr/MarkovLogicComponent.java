package binder.abstr;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;


import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.builders.TemporalUnionBuilder;
import beliefmodels.utils.DistributionUtils;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import binder.ml.MLException;
import binder.utils.MLNGenerator;
import binder.utils.MLNPreferences;
import cast.cdl.WorkingMemoryAddress;


public abstract class MarkovLogicComponent<T extends Belief> extends FakeComponent {

	
	// the directory in which to find the MLN code 
	protected static String markovlogicDir = "subarchitectures/binder/markovlogic/";

	// the command line for Alchemy inference over MLN
	protected String inferCmd = "tools/alchemy/bin/infer";

	// empty file for providing the evidence 
	protected String emptyFile = markovlogicDir + "empty.db";

	// the predicate to query
	private String query = "Outcome";

	// the result file
	protected String resultsFile = markovlogicDir + "unions.results";

	// minimum probability threshold
	public float lowestProbThreshold = 0.20f;
	
	// maximum number of alternatives to consider
	public int maxAlternatives = 1;
	
	// minimum difference between probabilities to trigger an explicit update
	public float minProbDifferenceForUpdate = 0.1f;

		
	
	/**
	 * Perform the generic inference operation for the given belief, and subsequently update
	 * the working memory with the outcome (and possibly also with updates on existing ones)
	 * 
	 * @param belief the new belief which was inserted
	 * @param beliefWMAddress the address of the belief
	 * @param prefs the preferences for the inference
	 */
	public List<Belief> performInference(T belief, WorkingMemoryAddress beliefWMAddress, MLNPreferences prefs)
		throws BeliefException {
		// extract the unions already existing in the binder WM
		Map<String, Belief> existingUnions = extractExistingUnions();
		
		Map<String, Belief> relevantUnions = selectRelevantUnions(existingUnions, belief);
		
		debug("nb existing unions: " + existingUnions.size());
		debug("nb relevant unions: " + relevantUnions.size());
		debug("tracking activated for belief: " + prefs.isTrackingActivated());
		
		if (relevantUnions.size() > 0 && 
				prefs.isTrackingActivated()) {
			
			debug("doing markov logic inference on belief "  + belief.id);
			debug("Markov Logic Network used: " + prefs.getFile_correlations());
			try {
			List<Belief> results = performMarkovLogicInference(belief, beliefWMAddress, relevantUnions, prefs);
			return results;
			}
			catch (BeliefException e) {
				return performDirectInference(belief, beliefWMAddress);
			}
		}
		
		else { 
			return performDirectInference(belief, beliefWMAddress);
		}
	}

	
	/**
	 * Perform the Markov Logic inference given a belief, a working memory address,
	 * the set of relevant unions, and the preferences, and return the combination of new
	 * beliefs to create, and existing beliefs to update
	 * 
	 * @param belief the basic belief
	 * @param beliefWMAddress the address
	 * @param relevantUnions the set of relevant unions on which to perform the match
	 * @param prefs the preferences
	 * @return the set of new or modified beliefs
	 * @throws BeliefException 
	 */
	
	private List<Belief> performMarkovLogicInference (T belief,
			WorkingMemoryAddress beliefWMAddress,
			Map<String, Belief> relevantUnions, MLNPreferences prefs) throws BeliefException {
		
		List<Belief> resultingBeliefs = new LinkedList<Belief>();
		
		// Create identifiers for each possible new union		
		Map<String, String> unionsMapping = createIdentifiersForNewUnions(relevantUnions);
		
		String newSingleUnionId = newDataID();
		
		// Write the Markov logic network to a file
		writeMarkovLogic(belief, relevantUnions, unionsMapping, newSingleUnionId, prefs);

		// run the alchemy inference
		HashMap<String,Float> inferenceResults = runAlchemyInference(prefs.getGeneratedMLNFile(), resultsFile);

			try { 

			log("filtering inference results to keep only the " + maxAlternatives + " best alternatives");
			HashMap<String,Float> filteredInferenceResults = filterInferenceResults(inferenceResults);

			// looping on the result
			for (String id : filteredInferenceResults.keySet()) {
				
				// if the result pertains to an already existing belief
				if (existsOnWorkingMemory(id)) {
					
					// checking the probability
					if (filteredInferenceResults.get(id) > lowestProbThreshold) {
						
						// if the probability is high enough, add it to the set of beliefs
						resultingBeliefs.add(getMemoryEntry
								(new WorkingMemoryAddress(id, BindingWorkingMemory.BINDER_SA), 
							TemporalUnionBelief.class));
					}
				}
				
				
				// if the belief does not exist, create a new one
				else {			
					
					// creating the new belief
					TemporalUnionBelief newBelief = 
						TemporalUnionBuilder.createNewSingleUnionBelief((MultiModalBelief) belief, beliefWMAddress, id);
					float existProb = 
						DistributionUtils.getExistenceProbability(newBelief) * filteredInferenceResults.get(id);
					
					if (newBelief.content instanceof DistributionWithExistDep) {
						DistributionUtils.setExistenceProbability(newBelief, existProb);
					}
					
					// checking the final probability
					if (existProb > lowestProbThreshold)  {

						resultingBeliefs.add(newBelief);
					}
				}
			}
						
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return resultingBeliefs;
	}


	/**
	 * Perform direct (i.e. non-MLN-based) inference on the beliefs
	 * 
	 * @param belief the belief
	 * @param beliefWMAddress the belief address
	 * @return the new belief to create
	 */
	private List<Belief> performDirectInference (T belief,
			WorkingMemoryAddress beliefWMAddress) {

		List<Belief> resultingBeliefs = new LinkedList<Belief>();
		try {
			log("no markov logic tracking for mmbelief " + belief.id);
			Belief union = TemporalUnionBuilder.createNewSingleUnionBelief
			((MultiModalBelief)belief, beliefWMAddress, newDataID());
				
			resultingBeliefs.add(union);
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return resultingBeliefs;
	}
	
	
	
	/**
	 * Run alchemy inference on a markov logic file, and outputting results in a file
	 * 
	 * @param mlnFile the markov logic file
	 * @param resultsFile the results file
	 * @return
	 * @throws BeliefException belief exception thrown
	 */
	public HashMap<String,Float> runAlchemyInference(String mlnFile, String resultsFile) throws BeliefException {

		Runtime run = Runtime.getRuntime(); 
		log("Now running Alchemy...");
		try {
			String[] args = {inferCmd, "-i", mlnFile, "-e", emptyFile, "-r", resultsFile, "-q", query, "-maxSteps", "50"};
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
	 
	/**
	 * Create identifiers for new unions
	 * NOTE: this is currently a hack
	 * 
	 * @param relevantUnions the set of relevant unions on which to base these
	 * 		identifiers
	 * @return returning the mapping from old to new identifiers
	 *  
	 */
	private Map<String, String> createIdentifiersForNewUnions(
			Map<String, Belief> relevantUnions) {
		Map<String,String> unionsMapping = new HashMap<String,String>();
		
		for (String existingUnionId : relevantUnions.keySet()) {
			String newUnionId = newDataID();
			unionsMapping.put(existingUnionId, existingUnionId);
		}
		
		log("newly created union ids: " + unionsMapping.keySet().toString());
		return unionsMapping;
	}
	

	/**
	 * Write the markov logic file for a new given belief, the set of relevant unions, the 
	 * unions mapping, the new union id, and the set of preferences
	 * 
	 * @param mmbelief the new multi-modal belief
	 * @param relevantUnions the set of relevant unions
	 * @param unionsMapping the identifiers mapping
	 * @param newSingleUnionId the new union id
	 * @param prefs the set of preferences
	 */
	private void writeMarkovLogic(T mmbelief,
			Map<String, Belief> relevantUnions,
			Map<String, String> unionsMapping, String newSingleUnionId, MLNPreferences prefs) {
		
		try {
			MLNGenerator gen = new MLNGenerator(prefs);
			gen.writeMLNFile(mmbelief, relevantUnions.values(), unionsMapping, 
					newSingleUnionId, prefs.getGeneratedMLNFile());
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
	

	protected abstract HashMap<String, Belief> extractExistingUnions();

	/**
	 * Only selecting the unions which are originating from the same subarchitecture as the current 
	 * mmbelief
	 * 
	 * @param existingUnions
	 * @param belief
	 * @return
	 * @throws BeliefException
	 */
	abstract protected Map<String,Belief> selectRelevantUnions 
		(Map<String, Belief> existingUnions, T belief) throws BeliefException  ;


	protected MultiModalBelief duplicateBelief(MultiModalBelief b) throws BeliefException {
		return new MultiModalBelief(b.frame, b.estatus, 	b.id,b.type, 
				FeatureContentUtils.duplicateContent(b.content), b.hist);
	}
}
