package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.ConnectivityPathProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

public class ConnectivityTransferFunction extends
		DependentDiscreteTransferFunction<ConnectivityPathProperty, PerceptBelief> {

	static final String ATTR_CONNECTED = "connected";

	/**
	 * @param perceptBeliefs
	 */
	public ConnectivityTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> perceptBeliefs) {
		super(component, perceptBeliefs, Logger
				.getLogger(ConnectivityTransferFunction.class), PerceptBelief.class);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seebinder.components.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, ConnectivityPathProperty from) {
		PerceptBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, final ConnectivityPathProperty from)
			throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

		WorkingMemoryAddress wmaPlace1 = getReferredBelief(new PlaceMatchingFunction(
				from.place1Id));
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(new PlaceMatchingFunction(
				from.place2Id));
		result.put("val0", WMPointer.create(wmaPlace1).getAsFormula());
		result.put("val1", WMPointer.create(wmaPlace2).getAsFormula());
		result.put("connected", BoolFormula.create(true).getAsFormula());
		return result;
	}

}
