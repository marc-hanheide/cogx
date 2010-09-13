package eu.cogx.perceptmediator.components;

import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceContainmentAgentTransferFunction;
import Ice.ObjectImpl;
import SpatialProperties.PlaceContainmentAgentProperty;

public class AgentMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, PlaceContainmentAgentProperty.class,
				new PlaceContainmentAgentTransferFunction(this, perceptBeliefsView));
	}

}
