/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.sharedtracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.WMTrackedBeliefMap;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.SharedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.ThresholdedBeliefMatcher;
import eu.cogx.percepttracker.WMTracker;

/**
 * @author Nick Hawes, but copied from Marc Hanheide (marc@hanheide.de)
 * 
 */
public class GeorgeSharedBeliefTracker extends ManagedComponent {


	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
	.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE);

	WMTracker<GroundedBelief, SharedBelief> tracker = null;

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
			tracker = WMTracker.create(this, GroundedBelief.class,
					SharedBelief.class, new ThresholdedBeliefMatcher(types, wm2wmMap,
							GroundedBelief.class, SharedBelief.class),
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
