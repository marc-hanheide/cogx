package binder.perceptmediator.components;

import Ice.ObjectImpl;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.ComaRoomRelationTransferFunction;

import comadata.ComaRoom;

public class RoomRelationMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, ComaRoom.class,
				new ComaRoomRelationTransferFunction(this, perceptBeliefsView));
	}

}
