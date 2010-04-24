package binder.perceptmediator.components;

import Ice.ObjectImpl;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.PerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.ComaRoomTransferFunction;

import comadata.ComaRoom;

public class RoomMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, ComaRoom.class, new ComaRoomTransferFunction(this));
	}

}
