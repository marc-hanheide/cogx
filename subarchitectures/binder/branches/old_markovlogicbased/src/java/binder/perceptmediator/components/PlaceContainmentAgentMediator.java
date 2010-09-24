package binder.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.PlaceContainmentAgentProperty;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.PlaceContainmentAgentTransferFunction;

public class PlaceContainmentAgentMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, PlaceContainmentAgentProperty.class, new PlaceContainmentAgentTransferFunction(this, perceptBeliefsView));
	}

}
