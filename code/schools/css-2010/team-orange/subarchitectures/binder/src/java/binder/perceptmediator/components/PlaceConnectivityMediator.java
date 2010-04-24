package binder.perceptmediator.components;

import Ice.ObjectImpl;
import SpatialProperties.ConnectivityPathProperty;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.ConnectivityTransferFunction;

public class PlaceConnectivityMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, ConnectivityPathProperty.class, new ConnectivityTransferFunction(this, perceptBeliefsView));
	}

}
