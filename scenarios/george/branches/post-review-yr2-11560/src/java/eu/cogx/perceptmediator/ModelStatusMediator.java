package eu.cogx.perceptmediator;

import VisionData.VisualConceptModelStatus;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;

public class ModelStatusMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<VisualConceptModelStatus, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this,
				VisualConceptModelStatus.class, GroundedBelief.class,
				new ModelStatusTransferFunction(this));
	}

}
