package binder.testing;


import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.utils.FeatureContentUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class BeliefUpdateTest extends AbstractBinderTest {

	
	 
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
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

		if (newBelief.id.equals("0:6")) {
			log("content of belief " + newBelief.id + ": " + ((StringValue)FeatureContentUtils.getValuesInBelief(newBelief, "colour").get(0).val).val);
		}
		
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

	@Override
	public void startTest() {
		try {

	
			 
			CondIndependentDistribs featDistrib = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			BasicProbDistribution colourDistrib = 
				BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("colour", new FeatureValueProbPair(new StringValue("red"), 0.8f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(featDistrib, colourDistrib);
					
			DistributionWithExistDep distrib = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, featDistrib);
			
			PerceptBelief p = PerceptBuilder.createNewPerceptBelief(newDataID(), "p2", "here", 
					this.getCASTTime(), distrib, PerceptBuilder.createHistory(new WorkingMemoryAddress("", "subarch1")));
			
			insertBeliefInWM(p);
			
			sleepComponent(3000);
			
			colourDistrib = 
				BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("colour", new FeatureValueProbPair(new StringValue("purple"), 0.8f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(featDistrib, colourDistrib);
			
			updateBeliefOnWM(p);

			
		} 
		catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
