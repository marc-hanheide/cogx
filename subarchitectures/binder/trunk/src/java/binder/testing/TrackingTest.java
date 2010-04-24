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
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.builders.PerceptUnionBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;


public class TrackingTest extends AbstractBinderTest {

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
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						checkIfSuccessful(_wmc);
					}
				}
		);
	}

 
   
	private void checkIfSuccessful(WorkingMemoryChange _wmc) {
		try { 
			CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					TemporalUnionBelief.class);
			TemporalUnionBelief newBelief = beliefData.getData();	
			nbUnionsOnWM++;
			log("union inserted on WM: " + newBelief.id);
			log("existence probability of union " + newBelief.id + ": " + ((DistributionWithExistDep)newBelief.content).existProb);
			
			log("number of unions received: " + nbUnionsOnWM);
		
			/**			TemporalUnionBelief pUnion = getMemoryEntry(new WorkingMemoryAddress("2:5", BindingWorkingMemory.BINDER_SA), TemporalUnionBelief.class);
			
		for (FeatureValueProbPair pair: FeatureContentUtils.getValuesInBelief(pUnion, "pointTo")) {
				if (pair.val.getClass().equals(PointerValue.class))
					log("belief " + pUnion.id + " pointing to belief "+ ((PointerValue)pair.val).beliefId.id + " with prob. " + pair.prob);
			} */

			if (nbUnionsOnWM==3) { 
				isTestSuccessful = true;
				isTestFinished = true;
				}
			log("Temporal Union correctly received!");
			
		}
		 catch (Exception e) {
				e.printStackTrace();
			}

	}


	@Override
	public void startTest() {
		
	

		try {
			
			CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));
	
			CondIndependentDistribs contentDistrib_b1 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			BasicProbDistribution typeDistrib_b1 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Person"), 0.85f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b1, typeDistrib_b1);

			BasicProbDistribution locationDistrib_b1 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("location", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Kitchen"), 0.91f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b1, locationDistrib_b1);
			
			ProbDistribution content_b1 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b1);
			
			PerceptBelief b1 = PerceptBuilder.createNewPerceptBelief(newDataID(), "test", "here", getCASTTime(), content_b1, hist);

			insertBeliefInWM(b1);
				
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		 
	
		
		////////////////////////////////
		
	 	sleepComponent(5000);

		try {
			
			CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress("ddfsadsf","vision"));
			
			CondIndependentDistribs contentDistrib_b2 = BeliefContentBuilder.createNewCondIndependentDistribs();
						
			BasicProbDistribution typeDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("type", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Person"), 0.89f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, typeDistrib_b2);

			BasicProbDistribution locationDistrib_b2 = BeliefContentBuilder.createNewFeatureDistributionWithSinglePair("location", 
					new FeatureValueProbPair(FeatureValueBuilder.createNewStringValue("Kitchen"), 0.89f));
			
			BeliefContentBuilder.putNewCondIndependentDistrib(contentDistrib_b2, locationDistrib_b2);
			
			ProbDistribution content_b2 = BeliefContentBuilder.createNewDistributionWithExistDep(0.9f, contentDistrib_b2);
			
			PerceptBelief b2 = PerceptBuilder.createNewPerceptBelief(newDataID(), "robot", "here", getCASTTime(), content_b2, hist);

			insertBeliefInWM(b2);		
			

		}
 
		catch (BeliefException e) {
			isTestFinished = true;
			isTestSuccessful = false;
			reasonForFailure = e.getMessage();
		}  catch (AlreadyExistsOnWMException e) {
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
