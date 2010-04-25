/**
 * 
 */
package binder.perceptmediator.transferfunctions.abstr;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import SpatialData.Place;
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
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;

/**
 * This is an abstract class for the most simple {@link TransferFunction} to
 * establish a mapping between input percepts (of generic type From) and
 * {@link PerceptBelief}. all it does is creating discrete FeatureDistribution
 * of an {@link CondIndependentDistribs} in a belief. This abstract
 * implementation requires the specific mapping between percepts and belief to
 * be implemented in a subclass by implementing getFeatureValueMapping.
 * 
 * @author marc
 * 
 * @param <From>
 *            type we generate beliefs from
 */
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl>
		extends CASTHelper implements TransferFunction<From, PerceptBelief> {

	/**
	 * constructor
	 * 
	 * @param component
	 */
	public SimpleDiscreteTransferFunction(ManagedComponent component,
			Logger logger) {
		super(component);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMEntrySynchronizer.TransferFunction#transform
	 * (cast.cdl.WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public boolean transform(WorkingMemoryChange wmc, From from,
			PerceptBelief perceptBelief) {
		assert (perceptBelief != null);
		assert (perceptBelief.content != null);
		assert (perceptBelief.content instanceof CondIndependentDistribs);
		{
			String offsprings = "";
			for (WorkingMemoryAddress offspringWMA : ((CASTBeliefHistory) perceptBelief.hist).offspring) {
				offsprings += " " + offspringWMA.id;
			}
			component.println("offspring BEFORE transformation occurs:"
					+ offsprings);
		}

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
			component.logException(e);
		}
		{
			String offsprings = "";
			for (WorkingMemoryAddress offspringWMA : ((CASTBeliefHistory) perceptBelief.hist).offspring) {
				offsprings += " " + offspringWMA.id;
			}
			component.println("offspring AFTER transformation occured:"
					+ offsprings);
		}
		return true;
	}

	/**
	 * helper method to encapsulate all the create of a discrete feature in
	 * binding
	 * 
	 * @param features
	 *            the distribution of features
	 * @param key
	 *            the feature name to be written
	 * @param value
	 *            the {@link FeatureValue} to be written
	 */
	protected void putDiscreteFeature(CondIndependentDistribs features,
			String key, FeatureValue value) {
		List<FeatureValueProbPair> values = new LinkedList<FeatureValueProbPair>();
		values.add(new FeatureValueProbPair(value, 1.0f));
		BasicProbDistribution cdistrib;
		try {
			cdistrib = BeliefContentBuilder.createNewFeatureDistribution(key,
					values);
			BeliefContentBuilder.putNewCondIndependentDistrib(features,
					cdistrib);
		} catch (BeliefException e) {
			logger.error("Belief exception", e);
		}

	}

	/**
	 * the abstract method
	 * 
	 * @param wmc
	 *            the {@link WorkingMemoryChange} that caused the update
	 * @param from
	 *            the data entry that caused the update
	 * @return a {@link Map} of feature names and {@link FeatureValue} that will
	 *         be written to the belief.
	 * @throws InterruptedException
	 * @throws BeliefException
	 */
	protected abstract Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, From from) throws InterruptedException,
			BeliefException;

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
			CASTBeliefHistory hist = PerceptBuilder
					.createNewPerceptHistory(wmc.address);

			// always create a CondIndependentDistribs
			CondIndependentDistribs features = BeliefContentBuilder
					.createNewCondIndependentDistribs();

			PerceptBelief pb;

			pb = PerceptBuilder.createNewPerceptBelief(idToCreate.id,
					getBeliefTypeFromCastType(wmc.type), "here", CASTUtils
							.getTimeServer().getCASTTime(), features, hist);
			return pb;
		} catch (BeliefException e) {
			component.logException(e);
			return null;
		}

	}

	public static String getBeliefTypeFromCastType(String casttype) {
		StringTokenizer st = new StringTokenizer(casttype, ":");
		String type = casttype;
		while (st.hasMoreTokens())
			type = st.nextToken();
		return type;
	}

	public static Object getBeliefTypeFromCastType(Class<? extends Ice.Object> class1) {
		return getBeliefTypeFromCastType(CASTUtils.typeName(class1));
	}
}
