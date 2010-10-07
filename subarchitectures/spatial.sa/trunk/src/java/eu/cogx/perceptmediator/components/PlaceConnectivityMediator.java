package eu.cogx.perceptmediator.components;

import SpatialProperties.ConnectivityPathProperty;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ConnectivityTransferFunction;

public class PlaceConnectivityMediator extends
		ReferringPerceptMediatorComponent<PerceptBelief> {
 
	public PlaceConnectivityMediator() {
		super(PerceptBelief.class);
	}

	@Override
	protected PerceptBindingMediator<ConnectivityPathProperty, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this,
				ConnectivityPathProperty.class, PerceptBelief.class,
				new ConnectivityTransferFunction(this, allBeliefs));
	}

}
