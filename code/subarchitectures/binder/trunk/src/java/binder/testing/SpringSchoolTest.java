package binder.testing;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import binder.arch.BindingWorkingMemory;
import cast.cdl.WorkingMemoryAddress;

public class SpringSchoolTest extends AbstractBinderTest {

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
	
	 
			
			CASTBeliefHistory hist_b2 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));
	
			CondIndependentDistribs contentDistrib_b2 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			BasicProbDistribution typeDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("object"), 0.89f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, typeDistrib_b2);

			BasicProbDistribution locationDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.93f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, locationDistrib_b2);
			
			ProbDistribution content_b2 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b2);
			
			PerceptBelief b2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b2, hist_b2);

			insertBeliefInWM(b2);
				
		
			
			CASTBeliefHistory hist_b3 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("aa","vision"));
	
			CondIndependentDistribs contentDistrib_b3 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			BasicProbDistribution typeDistrib_b3 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("object"), 0.95f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b3, typeDistrib_b3);

			BasicProbDistribution locationDistrib_b3 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.95f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b3, locationDistrib_b3);
			
			ProbDistribution content_b3 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b3);
			
			PerceptBelief b3 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b3, hist_b3);

			sleepComponent(5000);
			
			insertBeliefInWM(b3);
				
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

}
