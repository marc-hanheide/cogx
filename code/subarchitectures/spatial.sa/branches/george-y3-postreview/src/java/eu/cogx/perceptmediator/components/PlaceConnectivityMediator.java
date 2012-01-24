package eu.cogx.perceptmediator.components;

import SpatialProperties.ConnectivityPathProperty;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ConnectivityTransferFunction;

public class PlaceConnectivityMediator extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public PlaceConnectivityMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<ConnectivityPathProperty, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this,_toSA,
				ConnectivityPathProperty.class, GroundedBelief.class,
				new ConnectivityTransferFunction(this, allBeliefs));
	}

}
