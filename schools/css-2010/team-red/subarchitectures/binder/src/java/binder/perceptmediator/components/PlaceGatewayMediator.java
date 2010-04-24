package binder.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.GatewayPlaceProperty;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.GatewayTransferFunction;

public class PlaceGatewayMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, GatewayPlaceProperty.class,
				new GatewayTransferFunction(this, perceptBeliefsView));
	}

}
