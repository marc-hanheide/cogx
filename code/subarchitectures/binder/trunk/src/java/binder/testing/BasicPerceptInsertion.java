package binder.testing;

import cast.cdl.WorkingMemoryPointer;
import binder.arch.BinderException;
import binder.autogen.Feature;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.DiscreteDistribution;
import binder.autogen.distribs.FormulaProbPair;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.epstatus.EpistemicStatus;
import binder.autogen.framing.SpatioTemporalFrame;
import binder.autogen.history.BeliefHistory;
import binder.autogen.history.PerceptHistory;
import binder.builders.BeliefContentBuilder;
import binder.builders.FormulaBuilder;
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
	public void startTest() {
		
		try {
		// constructing the percept
		
		CondIndependentDistribs features = BeliefContentBuilder.createNewCondIndependentDistribs();
		
		FormulaProbPair[] cpairs = new FormulaProbPair[2];
		cpairs[0] = new FormulaProbPair(FormulaBuilder.createNewElementaryFormula("red"), 0.7f);
		cpairs[1] = new FormulaProbPair(FormulaBuilder.createNewElementaryFormula("pink"), 0.3f);
		
		DiscreteDistribution cdistrib = BeliefContentBuilder.createNewDiscreteDistributionOverFeature(Feature.Colour, cpairs, true);
		
		BeliefContentBuilder.addCondIndependentDistrib(features, cdistrib);
		
		ProbDistribution beliefcontent = BeliefContentBuilder.createBeliefContentWithExistDep(0.8f, features);
		
		PerceptHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryPointer());
		
		PerceptBelief belief = PerceptBuilder.createNewPerceptBelief("here", this.getCASTTime(), beliefcontent, hist);
		
		isTestFinished = true;
		isTestSuccessful = true;
		
		}
		
		catch (BinderException e) {
			isTestFinished = true;
			isTestSuccessful = false;
			reasonForFailure = e.getMessage();
		}
		
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
	public String getReasonForFailure() {
		// TODO Auto-generated method stub
		return null;
	}



	
}
