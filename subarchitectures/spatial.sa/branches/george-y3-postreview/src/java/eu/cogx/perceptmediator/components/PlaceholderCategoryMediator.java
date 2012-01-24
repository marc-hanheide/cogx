package eu.cogx.perceptmediator.components;

import SpatialProperties.RoomCategoryPlaceholderProperty;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.PlaceholderCategoryTransferFunction;

public class PlaceholderCategoryMediator extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public PlaceholderCategoryMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<RoomCategoryPlaceholderProperty, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this,_toSA,
				RoomCategoryPlaceholderProperty.class, GroundedBelief.class,
				new PlaceholderCategoryTransferFunction(this, allBeliefs));
	}

}
