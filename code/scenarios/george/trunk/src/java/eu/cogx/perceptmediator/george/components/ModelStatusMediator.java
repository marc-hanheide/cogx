package eu.cogx.perceptmediator.george.components;

import VisionData.VisualConceptModelStatus;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.george.transferfunctions.ModelStatusTransferFunction;

public class ModelStatusMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<VisualConceptModelStatus, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this, _toSA,
				VisualConceptModelStatus.class, GroundedBelief.class,
				new ModelStatusTransferFunction(this));
	}


}
