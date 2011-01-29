package eu.cogx.perceptmediator.components;

import SpatialData.Place;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;

public class PlaceMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<Place, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, Place.class,
				GroundedBelief.class, new PlaceTransferFunction(this));
	}

}
