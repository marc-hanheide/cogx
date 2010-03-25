package binder.testing;

import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.FeatureValueDistribution;
import binder.autogen.distribs.FeatureValueProbPair;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.featurecontent.Feature;
import binder.autogen.history.PerceptHistory;
import binder.builders.BeliefContentBuilder;
import binder.builders.FeatureValueBuilder;
import binder.builders.PerceptBuilder;

public class BasicPerceptInsertion extends AbstractBinderTest {

	String name = "basic percept construction and insertion";
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
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						checkIfSuccessful(_wmc);
					}
				}
		);
	}



	private void checkIfSuccessful(WorkingMemoryChange _wmc) {
		isTestSuccessful = true;
		isTestFinished = true;
		log("WOOOHHOOO!");
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

			insertPerceptInWM(belief);

		}

		catch (BinderException e) {
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
