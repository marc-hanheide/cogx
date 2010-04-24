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
import beliefmodels.builders.PerceptUnionBuilder;


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
			
			CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","haptic"));
	
			// First union
			CondIndependentDistribs cdistrib_b1 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			List<FeatureValueProbPair> shapePairs_b1 = new LinkedList<FeatureValueProbPair>();
			shapePairs_b1.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Cyl"), 0.75f));
			shapePairs_b1.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Sphe"), 0.15f));
			
			BasicProbDistribution shapeDistrib_b1 = BeliefContentBuilder.createNewFeatureDistribution("shape", shapePairs_b1);		
			BeliefContentBuilder.putNewCondIndependentDistrib(cdistrib_b1, shapeDistrib_b1);
			
			ProbDistribution content_b1 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, cdistrib_b1);
			
			PerceptBelief b1 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b1, hist);

			insertBeliefInWM(b1);
			
			
			CondIndependentDistribs cdistrib_b2 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			List<FeatureValueProbPair> shapePairs_b2 = new LinkedList<FeatureValueProbPair>();
			shapePairs_b2.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Sphe"), 0.85f));
			
			BasicProbDistribution shapeDistrib_b2 = BeliefContentBuilder.createNewFeatureDistribution("shape", shapePairs_b2);		
			BeliefContentBuilder.putNewCondIndependentDistrib(cdistrib_b2, shapeDistrib_b2);
			
			ProbDistribution content_b2 = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, cdistrib_b2);
			
			PerceptBelief b2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b2, hist);
			
			insertBeliefInWM(b2);
			
						
			
			CondIndependentDistribs cdistrib_b3 = BeliefContentBuilder.createNewCondIndependentDistribs();
			
			ProbDistribution content_b3 = BeliefContentBuilder.createNewDistributionWithExistDep(0.005f, cdistrib_b3);

			PerceptBelief b3 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b3, hist);

			insertBeliefInWM(b3);
			
			
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		try {
			
			CondIndependentDistribs features = BeliefContentBuilder.createNewCondIndependentDistribs();
					
			List<FeatureValueProbPair> labelPairs = new LinkedList<FeatureValueProbPair>();
			labelPairs.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Mug"), 0.9f));
			labelPairs.add(new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Ball"), 0.05f));
			
			BasicProbDistribution labelDistrib = BeliefContentBuilder.createNewFeatureDistribution("label", labelPairs);		
			BeliefContentBuilder.putNewCondIndependentDistrib(features, labelDistrib);

			ProbDistribution beliefcontent = BeliefContentBuilder.createNewDistributionWithExistDep(0.85f, features);

			CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ssfsfasd","vision"));

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
