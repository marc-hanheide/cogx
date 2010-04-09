/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.featurecontent.Feature;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.PerceptHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.EpistemicStatusBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.builders.SpatioTemporalFrameBuilder;
import binder.arch.BinderException;




/**
 * @author marc
 *
 */
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends PerceptBelief> implements TransferFunction<From, To> {

	/* (non-Javadoc)
	 * @see binder.components.perceptmediator.TransferFunction#transform(Ice.ObjectImpl)
	 */
	@Override
	public void transform(From from, To perceptBelief) throws BinderException, InterruptedException {
		assert(perceptBelief != null);
	
		CondIndependentDistribs features = BeliefContentBuilder.createNewCondIndependentDistribs();
		Map<Feature, FeatureValue> mapping = getFeatureValueMapping(from);
		for (Entry<Feature, FeatureValue> fvm : mapping.entrySet()) {
			FeatureValueProbPair[] values = new FeatureValueProbPair[1];
			values[0] = new FeatureValueProbPair(fvm.getValue(), 1.0f);
			FeatureValueDistribution cdistrib;
			try {
				cdistrib = BeliefContentBuilder.createNewFeatureValueDistribution(fvm.getKey(), values, true);
				BeliefContentBuilder.addCondIndependentDistrib(features, cdistrib);
			} catch (BeliefException e) {
				Logger.getLogger(SimpleDiscreteTransferFunction.class).error("Belief exception", e);
			}
		}
		perceptBelief.content = features;
	}

	abstract Map<Feature, FeatureValue> getFeatureValueMapping(From from) throws InterruptedException;

	/* (non-Javadoc)
	 * @see binder.components.perceptmediator.TransferFunction#createBelief(java.lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException, BeliefException {
		SpatioTemporalFrame frame = 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame("here", curTime, curTime);
		
		// constructing the epistemic status
		EpistemicStatus status;
			status = EpistemicStatusBuilder.createNewPrivateEpistemicStatus(EpistemicStatusBuilder.ROBOT_AGENT);

		PerceptHistory hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryPointer());

		PerceptBelief pb = new PerceptBelief(frame, status, id, null, hist);
		return pb;
	}

}
