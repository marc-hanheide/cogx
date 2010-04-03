/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.Map;
import java.util.Map.Entry;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;

import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.FeatureValueDistribution;
import binder.autogen.distribs.FeatureValueProbPair;
import binder.autogen.epstatus.EpistemicStatus;
import binder.autogen.featurecontent.Feature;
import binder.autogen.featurecontent.FeatureValue;
import binder.autogen.framing.SpatioTemporalFrame;
import binder.autogen.history.PerceptHistory;
import binder.builders.BeliefContentBuilder;
import binder.builders.EpistemicStatusBuilder;
import binder.builders.PerceptBuilder;
import binder.builders.SpatioTemporalFrameBuilder;



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
			FeatureValueDistribution cdistrib = BeliefContentBuilder.createNewFeatureValueDistribution(fvm.getKey(), values, true);
			BeliefContentBuilder.addCondIndependentDistrib(features, cdistrib);
		}
		perceptBelief.content = features;
	}

	abstract Map<Feature, FeatureValue> getFeatureValueMapping(From from) throws InterruptedException;

	/* (non-Javadoc)
	 * @see binder.components.perceptmediator.TransferFunction#createBelief(java.lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException {
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
