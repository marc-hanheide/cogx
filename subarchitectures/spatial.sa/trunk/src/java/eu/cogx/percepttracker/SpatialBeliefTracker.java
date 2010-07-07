/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.percepttracker;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import SpatialData.Place;
import SpatialProperties.GatewayPlaceProperty;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.WMTrackedBeliefMap;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class SpatialBeliefTracker extends ManagedComponent {

	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Place.class));
	private static final String GATEWAYTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils
					.typeName(GatewayPlaceProperty.class));
	private static final List<String> types = Arrays.asList(PLACETYPE,
			GATEWAYTYPE, "relation", "Robot");

	WMTracker<PerceptBelief, GroundedBelief> tracker = null;

	/**
	 * configure the component
	 *  --write-to-sa <subarchitectureID> the SA to write to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		PointerMap<WMTrackedBeliefMap> wm2wmMap;
		try {
			wm2wmMap = new PointerMap<WMTrackedBeliefMap>(this,
					WMTrackedBeliefMap.class);
			tracker = WMTracker.create(this, PerceptBelief.class,
					GroundedBelief.class, new FormulaMatcher(types, wm2wmMap),
					wm2wmMap, config.get("--write-to-sa"));
		} catch (InstantiationException e) {
			logException("cannot create PointerMap and tracker", e);
		} catch (IllegalAccessException e) {
			logException("cannot create PointerMap and tracker", e);
		}


	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		tracker.run();
	}

}
