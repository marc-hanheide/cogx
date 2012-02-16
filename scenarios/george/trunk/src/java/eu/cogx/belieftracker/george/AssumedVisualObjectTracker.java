package eu.cogx.belieftracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.AssumedToMergedBeliefMap;
import eu.cogx.beliefs.slice.AssumedBelief;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.ThresholdedBeliefMatcher;
import eu.cogx.percepttracker.WMTracker;
/**
 * @author Nick Hawes, but copied from Marc Hanheide (marc@hanheide.de)
 * 
 */
public class AssumedVisualObjectTracker extends ManagedComponent {

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE);

	WMTracker<AssumedBelief, MergedBelief> tracker = null;

	/**
	 * configure the component --write-to-sa <subarchitectureID> the SA to write
	 * to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {

		PointerMap<AssumedToMergedBeliefMap> wm2wmMap;
		try {
			wm2wmMap = new PointerMap<AssumedToMergedBeliefMap>(this,
					AssumedToMergedBeliefMap.class);
			tracker = WMTracker.create(this, AssumedBelief.class,
					MergedBelief.class,
					new ThresholdedBeliefMatcher<AssumedBelief, MergedBelief>(
							types, wm2wmMap, AssumedBelief.class,
							MergedBelief.class), wm2wmMap, config
							.get("--write-to-sa"));
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
