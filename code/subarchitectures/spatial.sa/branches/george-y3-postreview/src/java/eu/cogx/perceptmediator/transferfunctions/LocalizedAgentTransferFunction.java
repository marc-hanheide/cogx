/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author marc
 * 
 */
public class LocalizedAgentTransferFunction<To extends dBelief> extends
		DependentDiscreteTransferFunction<PlaceContainmentAgentProperty, To> {

	public static final String ROBOT = "Robot";
	public static final String AGENT_ID = "AgentId";
	public static final String IS_IN = "is-in";

	public LocalizedAgentTransferFunction(ManagedComponent component,
			WMView<To> allBeliefs, Class<To> classType) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedAgentTransferFunction.class), classType);
		// TODO Auto-generated constructor stub
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
	public To create(WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			PlaceContainmentAgentProperty from) {
		// TODO Auto-generated method stub
		To pb = super.create(idToCreate, wmc, from);
		pb.type = ROBOT;
		return pb;
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from)
			throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		try {
			long currentPlace = ((IntegerValue) from.mapValue).value;
			log("current place id of agent is " + currentPlace);
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace));
			log("  the corresponding GroundedBelief is " + placeWMA);
			result.put(AGENT_ID, IntFormula.create((int) from.agentID)
					.getAsFormula());
			result.put(IS_IN, WMPointer.create(placeWMA,
					CASTUtils.typeName(this.beliefClass)).getAsFormula());
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return result;
	}

}
