package eu.cogx.belieftracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.PrivateToAssumedBeliefMap;
import eu.cogx.beliefs.slice.PrivateBelief;
import eu.cogx.beliefs.slice.AssumedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.ThresholdedBeliefMatcher;
import eu.cogx.percepttracker.WMTracker;
/**
 * @author Nick Hawes, but copied from Marc Hanheide (marc@hanheide.de)
 * 
 */
public class PrivateVisualObjectTracker extends ManagedComponent {

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE);

	WMTracker<PrivateBelief, AssumedBelief> tracker = null;

	/**
	 * configure the component --write-to-sa <subarchitectureID> the SA to write
	 * to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {log("Private tracker running");
		PointerMap<PrivateToAssumedBeliefMap> wm2wmMap;
		try {
			wm2wmMap = new PointerMap<PrivateToAssumedBeliefMap>(this,
					PrivateToAssumedBeliefMap.class);
			tracker = WMTracker.create(this, PrivateBelief.class,
					AssumedBelief.class,
					new ThresholdedBeliefMatcher<PrivateBelief, AssumedBelief>(
							types, wm2wmMap, PrivateBelief.class,
							AssumedBelief.class), wm2wmMap, config
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
	protected void runComponent() {log("Private tracker running2");
		tracker.run();
	}

}
