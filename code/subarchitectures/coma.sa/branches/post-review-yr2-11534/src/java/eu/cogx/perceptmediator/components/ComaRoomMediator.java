package eu.cogx.perceptmediator.components;

import comadata.ComaRoom;

import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;

public class ComaRoomMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<ComaRoom, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, ComaRoom.class,
				GroundedBelief.class, new ComaRoomTransferFunction(this));
	}

}
