package eu.cogx.perceptmediator.components;

import SpatialData.Place;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;

public class PlaceMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<Place, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, Place.class,
				PerceptBelief.class, new PlaceTransferFunction(this));
	}

}
