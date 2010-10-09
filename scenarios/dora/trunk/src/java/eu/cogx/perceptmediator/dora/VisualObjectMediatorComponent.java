package eu.cogx.perceptmediator.dora;

import VisionData.VisualObject;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class VisualObjectMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public VisualObjectMediatorComponent() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<VisualObject, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				PerceptBelief.class, new VisualObjectTransferFunction(this,
						allBeliefs));
	}

}
