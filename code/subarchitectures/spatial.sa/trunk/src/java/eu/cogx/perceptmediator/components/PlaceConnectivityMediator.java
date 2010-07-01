package eu.cogx.perceptmediator.components;

import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ConnectivityTransferFunction;
import Ice.ObjectImpl;
import SpatialProperties.ConnectivityPathProperty;

public class PlaceConnectivityMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, ConnectivityPathProperty.class, new ConnectivityTransferFunction(this, perceptBeliefsView));
	}

}
