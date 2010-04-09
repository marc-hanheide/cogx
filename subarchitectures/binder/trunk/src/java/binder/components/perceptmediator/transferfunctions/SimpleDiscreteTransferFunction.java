/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.PerceptHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.EpistemicStatusBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.builders.SpatioTemporalFrameBuilder;

/**
 * @author marc
 * 
 */
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends PerceptBelief>
		implements TransferFunction<From, To> {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#transform(Ice.ObjectImpl
	 * )
	 */
	@Override
	public void transform(From from, To perceptBelief) throws BeliefException,
			InterruptedException {
		assert (perceptBelief != null);
		assert(perceptBelief.content instanceof CondIndependentDistribs);

		CondIndependentDistribs features = (CondIndependentDistribs) perceptBelief.content;
		Map<String, FeatureValue> mapping = getFeatureValueMapping(from);
		for (Entry<String, FeatureValue> fvm : mapping.entrySet()) {
			FeatureValueProbPair[] values = new FeatureValueProbPair[1];
			values[0] = new FeatureValueProbPair(fvm.getValue(), 1.0f);
			FeatureValueDistribution cdistrib;
			try {
				cdistrib = BeliefContentBuilder
						.createNewFeatureValueDistribution(fvm.getKey(),
								values, true);
				BeliefContentBuilder.addCondIndependentDistrib(features,
						cdistrib);
			} catch (BeliefException e) {
				Logger.getLogger(SimpleDiscreteTransferFunction.class).error(
						"Belief exception", e);
			}
		}
		perceptBelief.content = features;
	}

	abstract Map<String, FeatureValue> getFeatureValueMapping(From from)
			throws InterruptedException;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief createBelief(String id, WorkingMemoryAddress srcAddr,
			CASTTime curTime) throws BeliefException {

		PerceptHistory hist = PerceptBuilder
				.createNewPerceptHistory(new WorkingMemoryPointer(srcAddr,
						"src"));

		CondIndependentDistribs features = BeliefContentBuilder
				.createNewCondIndependentDistribs();

		
		PerceptBelief pb = PerceptBuilder.createNewPerceptBelief(id, "here",
				curTime, features, hist);

		return pb;
	}

}
