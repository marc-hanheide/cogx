/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import castutils.facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public class LocalizedAgentTransferFunction extends
		DependentDiscreteTransferFunction<PlaceContainmentAgentProperty> {

	public LocalizedAgentTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedAgentTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from)
			throws BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		try {
			long currentPlace = ((IntegerValue) from.mapValue).value;
			log("current place id of agent is " + currentPlace);
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace));
			log("  the corresponding PerceptBelief is " + placeWMA);
			result.put("AgentId", FeatureValueBuilder
					.createNewIntegerValue((int) from.agentID));
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction
	 * #create(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryChange,
	 * Ice.ObjectImpl)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from) {
		// TODO Auto-generated method stub
		PerceptBelief pb = super.create(idToCreate, wmc, from);
		pb.type = "Robot";
		return pb;
	}

}
