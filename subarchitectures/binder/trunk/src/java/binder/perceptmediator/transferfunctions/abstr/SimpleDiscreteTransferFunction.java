/**
 * 
 */
package binder.perceptmediator.transferfunctions.abstr;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.PerceptBuilder;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;

/**
 * @author marc
 * 
 */
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl>
		implements TransferFunction<From, PerceptBelief>{

	/**
	 * @param logger
	 */
	public SimpleDiscreteTransferFunction(ManagedComponent component, Logger logger) {
		super();
		this.component = component;
		this.logger = logger;
	}

	protected ManagedComponent component;
	protected Logger logger;
	
	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#transform(Ice.ObjectImpl
	 * )
	 */
	@Override
	public boolean transform(WorkingMemoryChange wmc, From from, PerceptBelief perceptBelief) {
		assert (perceptBelief != null);
		assert (perceptBelief.content != null);
		assert (perceptBelief.content instanceof CondIndependentDistribs);

		CondIndependentDistribs features = (CondIndependentDistribs) perceptBelief.content;
		Map<String, FeatureValue> mapping;
		try {
			mapping = getFeatureValueMapping(wmc, from);
			for (Entry<String, FeatureValue> fvm : mapping.entrySet()) {
				putDiscreteFeature(features, fvm.getKey(), fvm.getValue());
			}
		} catch (InterruptedException e) {
			component.logException(e);
		} catch (BeliefException e) {
			component.logException(e);		}
		return true;
	}

	protected void putDiscreteFeature(CondIndependentDistribs features,
			String key, FeatureValue value) {
		List<FeatureValueProbPair> values = new LinkedList<FeatureValueProbPair>();
		values.add(new FeatureValueProbPair(value, 1.0f));
		BasicProbDistribution cdistrib;
		try {
			cdistrib = BeliefContentBuilder.createNewFeatureDistribution(key, values);
			// TODO Here the API should be used instead!
			features.distribs.put(cdistrib.key, cdistrib);
			//BeliefContentBuilder.putNewCondIndependentDistrib(features, cdistrib);
		} catch (BeliefException e) {
			logger.error(
					"Belief exception", e);
		}

	}

	protected abstract Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc, From from)
			throws InterruptedException, BeliefException;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, From from) {

		try {
		// create a simple history with just the link to the percept
		CASTBeliefHistory hist = PerceptBuilder.createNewPerceptHistory(wmc.address);

		// always create a CondIndependentDistribs
		CondIndependentDistribs features = BeliefContentBuilder
				.createNewCondIndependentDistribs();

		PerceptBelief pb;
			pb = PerceptBuilder.createNewPerceptBelief(idToCreate.id, wmc.type, "here",
					CASTUtils.getTimeServer().getCASTTime(), features, hist);
			return pb;
		} catch (BeliefException e) {
			component.logException(e);
			return null;
		}

	}

}
