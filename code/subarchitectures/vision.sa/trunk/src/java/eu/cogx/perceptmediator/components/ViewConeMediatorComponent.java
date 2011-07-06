package eu.cogx.perceptmediator.components;

import VisionData.ViewCone;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ViewConeTransferFunction;

public class ViewConeMediatorComponent extends
		ReferringPerceptMediatorComponent<PerceptBelief> {

	public ViewConeMediatorComponent() {
		super(PerceptBelief.class);
	}

	@Override
	protected PerceptBindingMediator<ViewCone, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, ViewCone.class,
				PerceptBelief.class, new ViewConeTransferFunction(this,
						this.allBeliefs));
	}

}
