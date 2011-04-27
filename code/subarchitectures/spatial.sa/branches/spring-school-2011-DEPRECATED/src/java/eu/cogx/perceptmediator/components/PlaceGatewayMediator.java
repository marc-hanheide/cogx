package eu.cogx.perceptmediator.components;

import SpatialProperties.GatewayPlaceProperty;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.GatewayTransferFunction;

public class PlaceGatewayMediator extends ReferringPerceptMediatorComponent<GroundedBelief> {

	public PlaceGatewayMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<GatewayPlaceProperty, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, GatewayPlaceProperty.class,
				GroundedBelief.class, new GatewayTransferFunction(this,
						allBeliefs));
	}

}
