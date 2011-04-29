package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.GatewayPlaceProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

public class GatewayTransferFunction extends
		DependentDiscreteTransferFunction<GatewayPlaceProperty, GroundedBelief> {

	/**
	 * @param GroundedBeliefs
	 */
	public GatewayTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> GroundedBeliefs) {
		super(component, GroundedBeliefs, Logger
				.getLogger(GatewayTransferFunction.class), GroundedBelief.class);

	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, final GatewayPlaceProperty from)
			throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				from.placeId));

		result.put("place", WMPointer.create(wmaPlace,
				CASTUtils.typeName(this.beliefClass)).getAsFormula());
		result.put("isGateway", BoolFormula.create(true).getAsFormula());
		return result;
	}

}
