package eu.cogx.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.PlaceContainmentAgentProperty;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;

public class AgentMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, PlaceContainmentAgentProperty.class,
				new LocalizedAgentTransferFunction(this, perceptBeliefsView));
	}

}
