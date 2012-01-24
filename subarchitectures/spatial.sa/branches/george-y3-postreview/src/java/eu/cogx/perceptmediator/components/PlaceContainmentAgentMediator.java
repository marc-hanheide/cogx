package eu.cogx.perceptmediator.components;

import SpatialProperties.PlaceContainmentAgentProperty;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceContainmentAgentTransferFunction;

public class PlaceContainmentAgentMediator extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public PlaceContainmentAgentMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<PlaceContainmentAgentProperty, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this,_toSA,
				PlaceContainmentAgentProperty.class, GroundedBelief.class,
				new PlaceContainmentAgentTransferFunction(this,
						allBeliefs));
	}

}
