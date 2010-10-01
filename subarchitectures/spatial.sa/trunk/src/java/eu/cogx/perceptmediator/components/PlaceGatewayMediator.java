package eu.cogx.perceptmediator.components;

import SpatialProperties.GatewayPlaceProperty;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.GatewayTransferFunction;

public class PlaceGatewayMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<GatewayPlaceProperty, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, GatewayPlaceProperty.class,
				PerceptBelief.class, new GatewayTransferFunction(this,
						perceptBeliefsView));
	}

}
