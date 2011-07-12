package eu.cogx.perceptmediator.dora;

import SpatialData.ViewPoint;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class ViewPointMediator extends ReferringPerceptMediatorComponent<GroundedBelief> {

	public ViewPointMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<ViewPoint, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, ViewPoint.class, GroundedBelief.class, new ViewPointTransferFunction(this, this.allBeliefs));
	}

}
