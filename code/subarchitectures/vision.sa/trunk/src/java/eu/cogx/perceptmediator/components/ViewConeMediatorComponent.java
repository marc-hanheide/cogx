package eu.cogx.perceptmediator.components;

import VisionData.ViewCone;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ViewConeTransferFunction;

public class ViewConeMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public ViewConeMediatorComponent() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<ViewCone, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, ViewCone.class,
				GroundedBelief.class, new ViewConeTransferFunction(this,
						this.allBeliefs));
	}

}
