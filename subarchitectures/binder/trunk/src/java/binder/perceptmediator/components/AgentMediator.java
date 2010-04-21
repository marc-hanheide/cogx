package binder.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.PlaceContainmentAgentProperty;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;

public class AgentMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, PlaceContainmentAgentProperty.class,
				new LocalizedAgentTransferFunction(this, perceptBeliefsView));
	}

}
