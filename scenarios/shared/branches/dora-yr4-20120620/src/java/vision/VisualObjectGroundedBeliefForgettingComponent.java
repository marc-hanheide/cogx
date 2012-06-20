package vision;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import VisionData.VisualObject;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.AbstractForgettingComponent;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectGroundedBeliefForgettingComponent extends
		AbstractForgettingComponent<GroundedBelief> {

	final Set<String> acceptedTypes = new HashSet<String>();

	@Override
	protected boolean shouldBeHandled(WorkingMemoryChange wmc)
			throws CASTException {
		dBelief bel = getMemoryEntry(wmc.address, dBelief.class);
		if (acceptedTypes.contains(bel.type)) {
			return true;
		}
		return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.AbstractForgettingComponent#configure(java.util
	 * .Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		super.configure(config);
		registeredTypes.add(GroundedBelief.class);
		acceptedTypes.add(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class));

	}

}
