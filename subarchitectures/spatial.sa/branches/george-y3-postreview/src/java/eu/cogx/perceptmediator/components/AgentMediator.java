package eu.cogx.perceptmediator.components;

import SpatialProperties.PlaceContainmentAgentProperty;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;

public class AgentMediator extends ReferringPerceptMediatorComponent<GroundedBelief> {

	public AgentMediator() {
		super(GroundedBelief.class);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected PerceptBindingMediator<PlaceContainmentAgentProperty, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, 
				PlaceContainmentAgentProperty.class, GroundedBelief.class,
				new LocalizedAgentTransferFunction<GroundedBelief>(this, allBeliefs, GroundedBelief.class));
	}

}
