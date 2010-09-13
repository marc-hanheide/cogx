package eu.cogx.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialData.Place;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;

public class PlaceMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, Place.class, new PlaceTransferFunction(this));
	}

}
