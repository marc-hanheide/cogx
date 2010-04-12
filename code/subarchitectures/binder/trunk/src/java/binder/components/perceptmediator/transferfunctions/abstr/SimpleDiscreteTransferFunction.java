/**
 * 
 */
package binder.components.perceptmediator.transferfunctions.abstr;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.history.PerceptHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.PerceptBuilder;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

/**
 * @author marc
 * 
 */
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends PerceptBelief>
		implements TransferFunction<From, To> {

	/**
	 * @param logger
	 */
	public SimpleDiscreteTransferFunction(Logger logger) {
		super();
		this.logger = logger;
	}

	Logger logger;
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
		assert (perceptBelief.content != null);
		assert (perceptBelief.content instanceof CondIndependentDistribs);

		CondIndependentDistribs features = (CondIndependentDistribs) perceptBelief.content;
		Map<String, FeatureValue> mapping = getFeatureValueMapping(from);
		for (Entry<String, FeatureValue> fvm : mapping.entrySet()) {
			putDiscreteFeature(features, fvm.getKey(), fvm.getValue());
		}
	}

	protected void putDiscreteFeature(CondIndependentDistribs features,
			String key, FeatureValue value) {
		List<FeatureValueProbPair> values = new LinkedList<FeatureValueProbPair>();
		values.add(new FeatureValueProbPair(value, 1.0f));
		FeatureValueDistribution cdistrib;
		try {
			cdistrib = BeliefContentBuilder.createNewFeatureValueDistribution(
					values, false);
			BeliefContentBuilder.putNewCondIndependentDistrib(features, key,
					cdistrib);
		} catch (BeliefException e) {
			logger.error(
					"Belief exception", e);
		}

	}

	protected abstract Map<String, FeatureValue> getFeatureValueMapping(From from)
			throws InterruptedException, BeliefException;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief createBelief(String id, WorkingMemoryAddress srcAddr,
			String type, CASTTime curTime) throws BeliefException {

		// create a simple history with just the link to the percept
		PerceptHistory hist = PerceptBuilder.createNewPerceptHistory(srcAddr);

		// always create a CondIndependentDistribs
		CondIndependentDistribs features = BeliefContentBuilder
				.createNewCondIndependentDistribs();

		PerceptBelief pb = PerceptBuilder.createNewPerceptBelief(id, type, "here",
				curTime, features, hist);

		return pb;
	}

}
