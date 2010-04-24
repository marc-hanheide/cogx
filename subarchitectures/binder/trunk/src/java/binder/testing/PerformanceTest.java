package binder.testing;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class PerformanceTest extends AbstractBinderTest {

	CASTTime initTime;

	
	
	int nbTempUnions = 0;

	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						nbTempUnions++;
						log("RECEIVED: " + nbTempUnions);
						if (nbTempUnions == 100) {
						CASTTime ftime = getCASTTime();
						log ("FINAL TIME: " + ftime.s + " secs and " + ftime.us + " us");
						log("diff: " + (ftime.s-initTime.s) + " secs and " + (ftime.us-initTime.us) + " us");
						}
					}
				}
		);
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
			
			CASTBeliefHistory hist_b1 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","spatial"));
	
			CondIndependentDistribs contentDistrib_b1 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			BasicProbDistribution typeDistrib_b1 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Place"), 0.85f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b1, typeDistrib_b1);
		
			ProbDistribution content_b1 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b1);
			
			PerceptBelief b1 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b1, hist_b1);

			insertBeliefInWM(b1);
	
			CASTBeliefHistory hist_b2;
			CondIndependentDistribs contentDistrib_b2;
			BasicProbDistribution typeDistrib_b2;
			ProbDistribution content_b2;
			PerceptBelief b2;
			
			initTime = getCASTTime();
			log("INITIAL TIME: " + initTime.s + " secs and " + initTime.us + " usecs");
			
		for (int i = 0; i < 100 ; i++) {
			
			log("I: " + i);
			
			hist_b2 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));
	
			contentDistrib_b2 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			typeDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("object"), 0.89f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, typeDistrib_b2);

			BasicProbDistribution locationDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.93f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, locationDistrib_b2);
			
			content_b2 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b2);
			
			b2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b2, hist_b2);

			insertBeliefInWM(b2);
				
		}
			
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
	}

}
