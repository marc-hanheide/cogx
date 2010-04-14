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
import cast.core.CASTData;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;


public class PerceptualGroupingTest extends AbstractBinderTest {

	String name = "step 1 (perceptual grouping) after percept construction and insertion";
	String description = "construct one percept with all necessary properties " +
	"(spatio-temporal frame, epistemic status, belief content, history), and insert it into the WM";


	boolean isTestFinished=false;
	boolean isTestSuccessful=false;
	String reasonForFailure = "";
	
	int nbUnionsOnWM  = 0;

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
		
		

		try {
			
			// First union
			PerceptUnionBelief u1 = new PerceptUnionBelief();
			CondIndependentDistribs cdistrib_u1 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			List<FeatureValueProbPair> shapePairs_u1 = new LinkedList<FeatureValueProbPair>();
			shapePairs_u1.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Cyl"), 0.75f));
			shapePairs_u1.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Sphe"), 0.15f));
			
			BasicProbDistribution shapeDistrib_u1 = BeliefContentBuilder.createNewFeatureDistribution("shape", shapePairs_u1);		
			BeliefContentBuilder.putNewCondIndependentDistrib(cdistrib_u1, shapeDistrib_u1);
			
			u1.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, cdistrib_u1);
			u1.id = newDataID();
			addToWorkingMemory(u1.id, u1);
			
			
			PerceptUnionBelief u2 = new PerceptUnionBelief();
			CondIndependentDistribs cdistrib_u2 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			List<FeatureValueProbPair> shapePairs_u2 = new LinkedList<FeatureValueProbPair>();
			shapePairs_u2.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Sphe"), 0.85f));
			
			BasicProbDistribution shapeDistrib_u2 = BeliefContentBuilder.createNewFeatureDistribution("shape", shapePairs_u2);		
			BeliefContentBuilder.putNewCondIndependentDistrib(cdistrib_u2, shapeDistrib_u2);
			
			u2.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, cdistrib_u2);
			u2.id = newDataID();
			addToWorkingMemory(u2.id, u2);
			
			
			PerceptUnionBelief u3 = new PerceptUnionBelief();
			CondIndependentDistribs cdistrib_u3 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			u3.content = BeliefContentBuilder.createNewDistributionWithExistDep(0.005f, cdistrib_u3);
			u3.id = newDataID();
			addToWorkingMemory(u3.id, u3);
			
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}



	private void checkIfSuccessful(WorkingMemoryChange _wmc) {
		try { 
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptUnionBelief.class);
			PerceptUnionBelief newBelief = beliefData.getData();	
			nbUnionsOnWM++;
			log("union inserted on WM: " + newBelief.id);
			log("existence probability of union " + newBelief.id + ": " + ((DistributionWithExistDep)newBelief.content).existProb);
			
			if (nbUnionsOnWM==7) { 
				isTestSuccessful = true;
				isTestFinished = true;
			}
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
			
			
			CondIndependentDistribs cdistrib = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			List<FeatureValueProbPair> labelPairs = new LinkedList<FeatureValueProbPair>();
			labelPairs.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Mug"), 0.9f));
			labelPairs.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Ball"), 0.05f));
			
			BasicProbDistribution labelDistrib = BeliefContentBuilder.createNewFeatureDistribution("label", labelPairs);		
			BeliefContentBuilder.putNewCondIndependentDistrib(cdistrib, labelDistrib);

			ProbDistribution beliefcontent = BeliefContentBuilder.createNewDistributionWithExistDep(0.85f, features);

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
