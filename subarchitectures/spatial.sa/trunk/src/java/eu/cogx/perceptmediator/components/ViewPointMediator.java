package eu.cogx.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialData.ViewPoint;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ViewPointTransferFunction;

public class ViewPointMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<ViewPoint, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, ViewPoint.class, GroundedBelief.class, new ViewPointTransferFunction(this));
	}

}
