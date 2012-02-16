package eu.cogx.belieftracker.george;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.slice.WMMap;
import eu.cogx.beliefs.slice.AssumedBelief;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.slice.VerifiedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.percepttracker.RRMergerMatcher;
import eu.cogx.percepttracker.WMMerger;
/**
 * @author Nick Hawes, but copied from Marc Hanheide (marc@hanheide.de)
 * 
 */
public class AssumedVerifiedVisualObjectMerger extends ManagedComponent {

	private static final String VISUALOBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final List<String> types = Arrays.asList(VISUALOBJECTTYPE);

	WMMerger<AssumedBelief, VerifiedBelief, MergedBelief> merger = null;

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
			merger = WMMerger.create(this, AssumedBelief.class, VerifiedBelief.class,
					MergedBelief.class,
					new RRMergerMatcher<AssumedBelief, VerifiedBelief, MergedBelief>(
							types, srcDestMap, AssumedBelief.class, VerifiedBelief.class,
							MergedBelief.class),
					new RRMergerMatcher<VerifiedBelief, AssumedBelief, MergedBelief>(
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
