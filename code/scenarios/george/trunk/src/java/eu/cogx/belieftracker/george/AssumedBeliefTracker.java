package eu.cogx.belieftracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.ProtoObject;
import VisionData.VisualConceptModelStatus;
import VisionData.VisualObject;
import VisionData.ViewCone;
import execution.slice.Robot;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.PrivateToAssumedBeliefMap;
import eu.cogx.beliefs.slice.AssumedBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.AlwaysFalseMatcher;
import eu.cogx.percepttracker.WMTracker;

/**
 * @author Nick Hawes, but copied from Marc Hanheide (marc@hanheide.de)
 * 
 */
public class AssumedBeliefTracker extends ManagedComponent {

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final String PROTOOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ProtoObject.class));

	private static final String VISUALCONETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ViewCone.class));

	private static final String ROBOTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Robot.class));

	private static final String MSTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils
					.typeName(VisualConceptModelStatus.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE,
			PROTOOBJECTTYPE, VISUALCONETYPE, ROBOTTYPE, MSTYPE);

	WMTracker<GroundedBelief, AssumedBelief> tracker = null;

	/**
	 * configure the component --write-to-sa <subarchitectureID> the SA to write
	 * to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		PointerMap<PrivateToAssumedBeliefMap> wm2wmMap;
		try {
			wm2wmMap = new PointerMap<PrivateToAssumedBeliefMap>(this,
					PrivateToAssumedBeliefMap.class);
			tracker = WMTracker.create(this, GroundedBelief.class,
					AssumedBelief.class,
					new AlwaysFalseMatcher<GroundedBelief, AssumedBelief>(
							types, wm2wmMap, GroundedBelief.class,
							AssumedBelief.class), wm2wmMap, config
							.get("--write-to-sa"));
			tracker.setShouldPropagateDeletion(true);
		} catch (InstantiationException e) {
			logException("cannot create PointerMap and tracker", e);
		} catch (IllegalAccessException e) {
			logException("cannot create PointerMap and tracker", e);
		}

	}

	@Override
	protected void start() {
		tracker.registerListeners();
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
