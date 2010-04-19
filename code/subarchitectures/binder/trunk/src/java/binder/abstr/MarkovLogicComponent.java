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
import java.util.Map.Entry;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.history.CASTBeliefHistory;
import binder.arch.BindingWorkingMemory;
import binder.utils.MLNGenerator;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public abstract class MarkovLogicComponent<T extends Belief> extends BeliefWriter {

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

	public float lowestProbThreshold = 0.08f;
	public int maxAlternatives = 2;
	public float minProbDifferenceForUpdate = 0.1f;

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
	 * @param percept
	 * @param perceptWMAddress
	 * @throws BeliefException
	 */
	public abstract void workingMemoryChangeInsert(Belief percept, WorkingMemoryAddress perceptWMAddress) throws BeliefException;

	/**
	 * 
	 * @param perceptWMAddress
	 */
	public abstract void workingMemoryChangeDelete(WorkingMemoryAddress perceptWMAddress);

	/**
	 * Add a change filter on the insertion of new percept beliefs on the binder working memory
	 */
	public void start() {
		// Insertion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(type.getClass(),
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							CASTData<? extends Belief> beliefData = getMemoryEntryWithData(_wmc.address, type.getClass());

							log("received a new percept: " + beliefData.getID());
							workingMemoryChangeInsert(beliefData.getData(), _wmc.address);
							log("grouping operation on percept " + beliefData.getID() + " now finished");
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
		);

		// Deletion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(type.getClass(),
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							workingMemoryChangeDelete(_wmc.address);
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
		);

		// Update
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(type.getClass(),
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {	
						try {
							workingMemoryChangeDelete(_wmc.address);
							CASTData<? extends Belief> beliefData = getMemoryEntryWithData(_wmc.address, type.getClass());

							log("received a new percept: " + beliefData.getID());
							workingMemoryChangeInsert(beliefData.getData(), _wmc.address);
							log("grouping operation on percept " + beliefData.getID() + " now finished");
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
		);
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
}
