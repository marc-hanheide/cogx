package eu.cogx.belieftracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import VisionData.ProtoObject;
import VisionData.ViewCone;
import execution.slice.Robot;

import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.WMMap;
import eu.cogx.beliefs.slice.AssumedBelief;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.slice.VerifiedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.RRMergeFunction;
import eu.cogx.percepttracker.WMSimpleMerger;
/**
 * 
 */
public class AssumedVerifiedVisualObjectMerger extends ManagedComponent {

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

  private static final String PROTOOBJECTTYPE = SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(CASTUtils.typeName(ProtoObject.class));
                         			
  private static final String VISUALCONETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ViewCone.class));
			
	private static final String ROBOTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Robot.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE,
								PROTOOBJECTTYPE, VISUALCONETYPE, ROBOTTYPE);

	WMSimpleMerger<AssumedBelief, VerifiedBelief, MergedBelief> merger = null;

	/**
	 * configure the component --write-to-sa <subarchitectureID> the SA to write
	 * to
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {log("WMMerger running");
		super.configure(config);
		PointerMap<WMMap> srcDestMap;
		PointerMap<WMMap> srcSrcMap;
		try {
			srcDestMap = new PointerMap<WMMap>(this, WMMap.class);
			srcSrcMap = new PointerMap<WMMap>(this, WMMap.class);
			merger = WMSimpleMerger.create(this, AssumedBelief.class, VerifiedBelief.class,
					MergedBelief.class,
					new RRMergeFunction<AssumedBelief, VerifiedBelief, MergedBelief>(
							types, srcDestMap, AssumedBelief.class, VerifiedBelief.class,
							MergedBelief.class),
					new RRMergeFunction<VerifiedBelief, AssumedBelief, MergedBelief>(
							types, srcDestMap, VerifiedBelief.class, AssumedBelief.class,
							MergedBelief.class), 		
							 srcDestMap, srcSrcMap, config
							.get("--write-to-sa"));
			merger.setShouldPropagateDeletion(true);
		} catch (InstantiationException e) {
			logException("cannot create pointer maps and merger", e);
		} catch (IllegalAccessException e) {
			logException("cannot create pointer maps and merger", e);
		}

		
		
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		log("WMMerger running1");
		merger.run();
	}

}
