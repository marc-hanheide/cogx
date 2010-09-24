package binder.perceptmediator.components;

import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.PerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.PlaceTransferFunction;
import Ice.ObjectImpl;
import SpatialData.Place;

public class PlaceMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, Place.class, new PlaceTransferFunction(this));
	}

}
