/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import org.apache.log4j.Logger;

import comadata.ComaRoom;

import VisionData.VisualObject;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.perceptmediator.attic.ComaRoomMatchingFunction;
import binder.perceptmediator.transferfunctions.abstr.LocalizedRelationTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.ObjectMatchingFunction;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 * 
 */
public class ComaRoomRelationTransferFunction extends
		LocalizedRelationTransferFunction<ComaRoom> {

	public ComaRoomRelationTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(ComaRoomRelationTransferFunction.class));
	}

	@Override
	protected ContentMatchingFunction<PerceptBelief> getMatchingFunction(
			String id) {
		return new ComaRoomMatchingFunction(id);
	}

}
