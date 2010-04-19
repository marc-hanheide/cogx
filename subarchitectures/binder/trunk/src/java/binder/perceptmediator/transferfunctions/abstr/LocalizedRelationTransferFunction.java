/**
 * 
 */
package binder.perceptmediator.transferfunctions.abstr;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement0;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement1;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import castutils.facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public abstract class LocalizedRelationTransferFunction<From extends Ice.ObjectImpl>
		extends DependentDiscreteTransferFunction<From> {

	public LocalizedRelationTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs, Logger logger) {
		super(component, allBeliefs, logger);

	}

	protected abstract ContentMatchingFunction<PerceptBelief> getMatchingFunction(
			String id);

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction
	 * #getFeatureValueMapping(cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, From from) throws InterruptedException,
			BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		try {
			Place place;
			place = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
					place.id));
			WorkingMemoryAddress wmaFrom = getReferredBelief(getMatchingFunction(wmc.address.id));
			result.put("is-in", FeatureValueBuilder.createNewBooleanValue(true));
			result.put(RelationElement0.value, FeatureValueBuilder
					.createNewStringValue(wmaFrom.id));
			result.put(RelationElement1.value, FeatureValueBuilder
					.createNewStringValue(wmaPlace.id));

		} catch (CASTException e) {
			component.logException("CASTException when mapping features, not all features might have been assigned", e);
		}
		return result;
	}

	/* (non-Javadoc)
	 * @see binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, From from) {
		PerceptBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

}
