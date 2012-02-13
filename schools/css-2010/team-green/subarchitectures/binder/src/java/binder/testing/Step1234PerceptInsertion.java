package binder.testing;

import java.util.LinkedList;
import java.util.List;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;


public class Step1234PerceptInsertion extends AbstractBinderTest {

	String name = "step 1,2,3 and 4 after percept construction and insertion";
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
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						checkIfSuccessful(_wmc);
					}
				}
		);
	}



	private void checkIfSuccessful(WorkingMemoryChange _wmc) {
		try { 
			CASTData<StableBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					StableBelief.class);
			StableBelief newBelief = beliefData.getData();	
			isTestSuccessful = true;
			isTestFinished = true;
			log("Stable belief correctly received!");
			
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

			List<FeatureValueProbPair> values = new LinkedList<FeatureValueProbPair>();
			values.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("red"), 0.7f));
			values.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("pink"), 0.3f));

			String colourfeat = "colour";
			BasicProbDistribution cdistrib = BeliefContentBuilder.createNewFeatureDistribution(colourfeat,values);

			BeliefContentBuilder.putNewCondIndependentDistrib(features, cdistrib);

			ProbDistribution beliefcontent = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, features);

			CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("",""));

			String id = newDataID();

			PerceptBelief belief = PerceptBuilder.createNewPerceptBelief(id, "test", "here", this.getCASTTime(), beliefcontent, hist);

			insertBeliefInWM(belief);

		} 

		catch (BeliefException e) {
			isTestFinished = true;
			isTestSuccessful = false;
			reasonForFailure = e.getMessage();
		} catch (AlreadyExistsOnWMException e) {
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