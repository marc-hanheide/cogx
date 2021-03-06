package binder.testing;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class SpringSchoolTest extends AbstractBinderTest {

	
	
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							
							CASTData<TemporalUnionBelief>[] unions = getWorkingMemoryEntries(TemporalUnionBelief.class);
							log("Current number of unions in WM: " + unions.length);
							if (FeatureContentUtils.getValuesInBelief(unions[0].getData(), "detected").size() > 0) 
							log("features in first union " + unions[0].getID() + ":  " + ((BooleanValue)FeatureContentUtils.getValuesInBelief(unions[0].getData(), "detected").get(0).val).val);
						} 
						catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							
							CASTData<TemporalUnionBelief>[] unions = getWorkingMemoryEntries(TemporalUnionBelief.class);
							log("Current number of unions in WM: " + unions.length);
							if (FeatureContentUtils.getValuesInBelief(unions[0].getData(), "detected").size() > 0) 
							log("features in first union " + unions[0].getID() + ":  " + ((BooleanValue)FeatureContentUtils.getValuesInBelief(unions[0].getData(), "detected").get(0).val).val);
						} 
						catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							
							CASTData<TemporalUnionBelief>[] unions = getWorkingMemoryEntries(TemporalUnionBelief.class);
							log("Current number of unions in WM: " + unions.length);
						} 
						catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
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
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Office"), 0.85f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b1, typeDistrib_b1);

			ProbDistribution content_b1 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b1);

			PerceptBelief b1 = PerceptBuilder.createNewPerceptBelief(newDataID(), "place", "here", getCASTTime(), content_b1, hist_b1);

			insertBeliefInWM(b1);



			CASTBeliefHistory hist_b2 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));

			CondIndependentDistribs contentDistrib_b2 = BeliefContentBuilder.createNewCondIndependentDistribs();

			BasicProbDistribution typeDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("label", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("3:2"), 0.89f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, typeDistrib_b2);

			BasicProbDistribution locationDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.93f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, locationDistrib_b2);
			
		//	BasicProbDistribution expected = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("expected", 
		//			new FeatureValueProbPair(FeatureValueBuilder.createNewBooleanValue(true), 1.0f));

		//	BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, expected);
			

			ProbDistribution content_b2 = BeliefContentBuilder.createNewDistributionWithExistDep(1.0f, contentDistrib_b2);

			PerceptBelief b2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "Object", "here", getCASTTime(), content_b2, hist_b2);

			insertBeliefInWM(b2);



			CASTBeliefHistory hist_b3 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("aa","vision"));

			CondIndependentDistribs contentDistrib_b3 = BeliefContentBuilder.createNewCondIndependentDistribs();

			BasicProbDistribution typeDistrib_b3 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("label", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("3:2"), 0.95f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b3, typeDistrib_b3);

			BasicProbDistribution locationDistrib_b3 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.95f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b3, locationDistrib_b3);

			ProbDistribution content_b3 = BeliefContentBuilder.createNewDistributionWithExistDep(1.0f, contentDistrib_b3);

			PerceptBelief b3 = PerceptBuilder.createNewPerceptBelief(newDataID(), "Object", "here", getCASTTime(), content_b3, hist_b3);

			insertBeliefInWM(b3);
			
			 

			CASTBeliefHistory hist_b4 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));

			CondIndependentDistribs contentDistrib_b4 = BeliefContentBuilder.createNewCondIndependentDistribs();

			BasicProbDistribution locationDistrib_b4 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.93f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b4, locationDistrib_b4);

			BasicProbDistribution detected = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("detected", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewBooleanValue(true), 1.0f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b4, detected);

			
			ProbDistribution content_b4 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b4);

			PerceptBelief b4 = PerceptBuilder.createNewPerceptBelief(newDataID(), "Person", "here", getCASTTime(), content_b4, hist_b4);

		//	insertBeliefInWM(b4);



			CASTBeliefHistory hist_b5 = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("aa","vision"));

			CondIndependentDistribs contentDistrib_b5 = BeliefContentBuilder.createNewCondIndependentDistribs();

			BasicProbDistribution locationDistrib_b5 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("is-in", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewPointerValue(new WorkingMemoryAddress(b1.id, BindingWorkingMemory.BINDER_SA)), 0.95f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b5, locationDistrib_b5);

	
			BasicProbDistribution distanceDistrib_b5 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("distance", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewFloatValue(0.67f), 1.0f));

			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b5, distanceDistrib_b5); 
			
		
			ProbDistribution content_b5 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b5);

			PerceptBelief b5 = PerceptBuilder.createNewPerceptBelief(newDataID(), "Person", "here", getCASTTime(), content_b5, hist_b5);

		//	insertBeliefInWM(b5);						

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

}
