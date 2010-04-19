package binder.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.PlaceContainmentAgentProperty;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.PerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.AgentTransferFunction;

public class AgentMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, PlaceContainmentAgentProperty.class,
				new AgentTransferFunction(this));
	}

}
