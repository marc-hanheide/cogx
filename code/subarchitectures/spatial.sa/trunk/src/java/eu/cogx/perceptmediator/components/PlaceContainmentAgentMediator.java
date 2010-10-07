package eu.cogx.perceptmediator.components;

import SpatialProperties.PlaceContainmentAgentProperty;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceContainmentAgentTransferFunction;

public class PlaceContainmentAgentMediator extends
		ReferringPerceptMediatorComponent<PerceptBelief> {

	public PlaceContainmentAgentMediator() {
		super(PerceptBelief.class);
	}

	@Override
	protected PerceptBindingMediator<PlaceContainmentAgentProperty, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this,
				PlaceContainmentAgentProperty.class, PerceptBelief.class,
				new PlaceContainmentAgentTransferFunction(this,
						allBeliefs));
	}

}
