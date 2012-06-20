/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.percepttracker.components;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.WMTrackedBeliefMap;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.FormulaMatcher;
import eu.cogx.percepttracker.WMTracker;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class VisualObjectTracker extends ManagedComponent {

	private static final String OBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));
	private static final List<String> types = Arrays.asList(OBJECTTYPE);

	WMTracker<PerceptBelief, GroundedBelief> tracker = null;

	/**
	 * configure the component --write-to-sa <subarchitectureID> the SA to write
	 * to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		PointerMap<WMTrackedBeliefMap> wm2wmMap;
		try {
			wm2wmMap = new PointerMap<WMTrackedBeliefMap>(this,
					WMTrackedBeliefMap.class);
			tracker = WMTracker
					.create(this, PerceptBelief.class, GroundedBelief.class,
							new FormulaMatcher<PerceptBelief, GroundedBelief>(
									types, wm2wmMap, PerceptBelief.class,
									GroundedBelief.class), wm2wmMap, config
									.get("--write-to-sa"));
			// TODO check whether we really want the visual objects to be
			// removed on deletion
			tracker.setShouldPropagateDeletion(true);
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
