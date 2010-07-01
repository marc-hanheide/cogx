package eu.cogx.perceptmediator.components;

import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.GatewayTransferFunction;
import Ice.ObjectImpl;
import SpatialProperties.GatewayPlaceProperty;

public class PlaceGatewayMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, GatewayPlaceProperty.class,
				new GatewayTransferFunction(this, perceptBeliefsView));
	}

}
