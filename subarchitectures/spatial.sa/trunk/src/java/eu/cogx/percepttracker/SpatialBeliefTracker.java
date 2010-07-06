/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.percepttracker;

import java.util.Arrays;
import java.util.List;

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

	final WMTracker<PerceptBelief, GroundedBelief> tracker;

	public SpatialBeliefTracker() throws InstantiationException,
			IllegalAccessException {
		List<String> types = Arrays.asList(PLACETYPE, GATEWAYTYPE, "relation",
				"Robot");
		PointerMap<WMTrackedBeliefMap> wm2wmMap;
		wm2wmMap = new PointerMap<WMTrackedBeliefMap>(this,
				WMTrackedBeliefMap.class);
		tracker = new WMTracker<PerceptBelief, GroundedBelief>(this,
				PerceptBelief.class, GroundedBelief.class, new FormulaMatcher(
						types, wm2wmMap), wm2wmMap);
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
