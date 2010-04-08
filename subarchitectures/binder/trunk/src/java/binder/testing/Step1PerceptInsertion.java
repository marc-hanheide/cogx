package binder.testing;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.Feature;
import beliefmodels.autogen.history.PerceptHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;


public class Step1PerceptInsertion extends AbstractBinderTest {

	String name = "step 1 (perceptual grouping) after percept construction and insertion";
	String description = "construct one percept with all necessary properties " +
	"(spatio-temporal frame, epistemic status, belief content, history), and insert it into the WM";


	boolean isTestFinished=false;
	boolean isTestSuccessful=false;
	String reasonForFailure = "";

	@Override
	public String getTestName() {
		return name;
	}

	@Override
	public String getTestDescription() {
		return description;
	}

	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						checkIfSuccessful(_wmc);
					}
				}
		);
	}



	private void checkIfSuccessful(WorkingMemoryChange _wmc) {
		try { 
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptUnionBelief.class);
			PerceptUnionBelief newBelief = beliefData.getData();	
			isTestSuccessful = true;
			isTestFinished = true;
			log("Percept Union correctly received!");
			
		}
		 catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			}
		 catch (UnknownSubarchitectureException e) {	
			e.printStackTrace();
		} 

	}


	@Override
	public void startTest() {

		try {

			CondIndependentDistribs features = BeliefContentBuilder.createNewCondIndependentDistribs();

			FeatureValueProbPair[] values = new FeatureValueProbPair[2];
			values[0] = new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("red"), 0.7f);
			values[1] = new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("pink"), 0.3f);

			FeatureValueDistribution cdistrib = BeliefContentBuilder.createNewFeatureValueDistribution(Feature.Colour, values, true);

			BeliefContentBuilder.addCondIndependentDistrib(features, cdistrib);

			ProbDistribution beliefcontent = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, features);

			PerceptHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryPointer());

			String id = newDataID();

			PerceptBelief belief = PerceptBuilder.createNewPerceptBelief(id, "here", this.getCASTTime(), beliefcontent, hist);

			insertBeliefInWM(belief);

		}

		catch (BeliefException e) {
			isTestFinished = true;
			isTestSuccessful = false;
			reasonForFailure = e.getMessage();
		}

	}


	@Override
	public boolean isTestFinished() {
		return isTestFinished;
	}

	@Override
	public boolean isTestSuccessful() {
		return isTestSuccessful;
	}


	@Override
	public String getReasonForFailure() {
		return reasonForFailure;
	}




}
