package eu.cogx.perceptmediator.components;

import SpatialProperties.PlaceContainmentAgentProperty;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;

public class AgentMediator extends ReferringPerceptMediatorComponent<PerceptBelief> {

	public AgentMediator() {
		super(PerceptBelief.class);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected PerceptBindingMediator<PlaceContainmentAgentProperty, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this,
				PlaceContainmentAgentProperty.class, PerceptBelief.class,
				new LocalizedAgentTransferFunction<PerceptBelief>(this, allBeliefs, PerceptBelief.class));
	}

}
