/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.perceptmediator.transferfunctions.abstr.LocalizedRelationTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.ObjectMatchingFunction;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 * 
 */
public class VisualObjectRelationTransferFunction extends
		LocalizedRelationTransferFunction<VisualObject> {

	public VisualObjectRelationTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(VisualObjectRelationTransferFunction.class));
	}

	@Override
	protected ContentMatchingFunction<PerceptBelief> getMatchingFunction(
			String id) {
		return new ObjectMatchingFunction(id);
	}

}
