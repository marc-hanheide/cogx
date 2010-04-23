package binder.testing;

import java.util.LinkedList;
import java.util.List;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;

public class BeliefPointerTest extends AbstractBinderTest {

	 
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						checkIfSuccessful(_wmc);
					}
				}
		);
	} 
	 
	   
	public void checkIfSuccessful(WorkingMemoryChange wmc) {
		
		CASTData<StableBelief> beliefData;
		try {
			beliefData = getMemoryEntryWithData(wmc.address,
					StableBelief.class);
			
			StableBelief newBelief = beliefData.getData();	 

		if (newBelief.id.equals("4:6")) {
				log("belief " + beliefData.getID() + " (offspring from " + ((CASTBeliefHistory)newBelief.hist).ancestors.get(0).id + 
						"), has a feature pointing to: " + ((PointerValue)FeatureContentUtils.getValuesInBelief(newBelief, "pointer").get(0).val).beliefId.id);
			} 
	/**	else  {
			if (existsOnWorkingMemory(new WorkingMemoryAddress("0:6", BindingWorkingMemory.BINDER_SA))) {
				StableBelief otherUnion = getMemoryEntry(new WorkingMemoryAddress("0:6", BindingWorkingMemory.BINDER_SA), StableBelief.class);
				log("belief " + otherUnion.id + " (offspring from " + ((CASTBeliefHistory)otherUnion.hist).ancestors.get(0).id + 
						"), has a feature pointing to: " + ((PointerValue)FeatureContentUtils.getValuesInBelief(otherUnion, "pointer").get(0).val).beliefId.id);				
			} 
		} */ 
		}  catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	} 
	 
	@Override
	public String getReasonForFailure() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getTestDescription() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isTestFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isTestSuccessful() {
		// TODO Auto-generated method stub
		return false;
	}

	String id_p2;
	
	@Override
	public void startTest() {
	
		try {

			
			CondIndependentDistribs featDistrib_p1 = BeliefContentBuilder.createNewCondIndependentDistribs();
			DistributionWithExistDep distrib_p1 = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, featDistrib_p1);
			
			PerceptBelief p1 = PerceptBuilder.createNewPerceptBelief(newDataID(), "p1", "here", 
					this.getCASTTime(), distrib_p1, PerceptBuilder.createHistory(new WorkingMemoryAddress("", "subarch1")));
	
			 
			CondIndependentDistribs featDistrib_p2 = BeliefContentBuilder.createNewCondIndependentDistribs();
			List<FeatureValueProbPair> pointerVals = new LinkedList<FeatureValueProbPair>();
			FeatureValueProbPair pointerVal1 = new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(
					new WorkingMemoryAddress(p1.id, BindingWorkingMemory.BINDER_SA)), 1.0f);
			pointerVals.add(pointerVal1);
			BasicProbDistribution pointerFeat = BeliefContentBuilder.createNewFeatureDistribution("pointer", pointerVals);
			BeliefContentBuilder.putNewCondIndependentDistrib(featDistrib_p2, pointerFeat);
					
			DistributionWithExistDep distrib_p2 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, featDistrib_p2);
			
			PerceptBelief p2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "p2", "here", 
					this.getCASTTime(), distrib_p2, PerceptBuilder.createHistory(new WorkingMemoryAddress("", "subarch1")));
			
			
			insertBeliefInWM(p1);

			sleepComponent(5000);

			insertBeliefInWM(p2);	

			

			id_p2 = p2.id;

			
		} 
		catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
