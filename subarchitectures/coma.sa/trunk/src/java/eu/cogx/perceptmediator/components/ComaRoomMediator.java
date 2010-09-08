package eu.cogx.perceptmediator.components;

import comadata.ComaRoom;

import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import Ice.ObjectImpl;
import SpatialData.Place;

public class ComaRoomMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, ComaRoom.class, new ComaRoomTransferFunction(this));
	}

}
