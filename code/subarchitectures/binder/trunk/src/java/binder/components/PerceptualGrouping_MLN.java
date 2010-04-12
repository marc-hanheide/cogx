package binder.components;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.PerceptUnionBuilder;
import beliefmodels.utils.DistributionUtils;
import binder.abstr.BeliefWriter;
import binder.abstr.MarkovLogicComponent;
import binder.arch.BindingWorkingMemory;
import binder.utils.FileUtils;
import binder.utils.MLNGenerator;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class PerceptualGrouping_MLN extends MarkovLogicComponent {

	String MLNFile = markovlogicDir + "grouping.mln";
	String resultsFile = markovlogicDir + "unions.results";

	@Override
	public void start() {
		
		insertExistingUnionsForTesting();
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);	
							log("received a new percept belief!");
							performPerceptualGrouping (beliefData.getData());
						}	
			
						 catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							}
						 catch (UnknownSubarchitectureException e) {	
							e.printStackTrace();
						} 
						 
					}
				}
		);
		
	
	}

	private void insertExistingUnionsForTesting() {
		try {
			PerceptUnionBelief u1 = new PerceptUnionBelief();
			u1.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, new BasicProbDistribution());
			u1.id = newDataID();
			PerceptUnionBelief u2 = new PerceptUnionBelief();
			u2.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, new BasicProbDistribution());
			u2.id = newDataID();	
			PerceptUnionBelief u3 = new PerceptUnionBelief();
			u3.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.05f, new BasicProbDistribution());
			u3.id = newDataID();
			addToWorkingMemory(u1.id, u1);
			addToWorkingMemory(u2.id, u2);
			addToWorkingMemory(u3.id, u3);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void performPerceptualGrouping(PerceptBelief b) {
	
		log("now starting perceptual grouping...");

		HashMap<String, PerceptUnionBelief> existingUnions = extractExistingUnions();
		
		Vector<String> newUnions = new Vector<String>();
		HashMap<String,String> linkToExistingUnions = new HashMap<String,String>();
		for (String existingUnionId : existingUnions.keySet()) {
			String newUnionId = newDataID();
			newUnions.add(newUnionId);
			linkToExistingUnions.put(newUnionId, existingUnionId);
		}
			 
		MLNGenerator.writeMLNFile(b, existingUnions.values(), newUnions, MLNFile);
		
		HashMap<String,Float> inferenceResults = runAlchemyInference(MLNFile, resultsFile);
	
		
		float perceptExistProb = DistributionUtils.getExistenceProbability(b);
				
		for (String id : inferenceResults.keySet()) {
			float prob = perceptExistProb * inferenceResults.get(id);
			log("prob of " + id + ": " + prob);
		}
		 
		for (String newUnionId: linkToExistingUnions.keySet()) {
			PerceptUnionBelief associatedExistingUnion = existingUnions.get(linkToExistingUnions.get(newUnionId));
			float unionCurrentExistProb = DistributionUtils.getExistenceProbability(associatedExistingUnion);
			float unionNewExistProb = (unionCurrentExistProb * (1-perceptExistProb)) + 
				(perceptExistProb * (1 - inferenceResults.get(newUnionId)) * unionCurrentExistProb);
			log("new prob for " +  linkToExistingUnions.get(newUnionId) + ": " + unionNewExistProb);
			
			DistributionUtils.setExistenceProbability(associatedExistingUnion, unionNewExistProb);
		}
		
		// here, filtering should take place
		 try {
		PerceptUnionBelief union = PerceptUnionBuilder.createNewSingleUnionBelief(b, newDataID());
		insertBeliefInWM(union);
		 }
		 catch (BeliefException e) {
				e.printStackTrace();
			} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}

	}
	
	
	
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
	
	
	
	
}
