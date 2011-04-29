package eu.cogx.perceptmediator.components;

import VisionData.VisualObject;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.VisualObjectTransferFunction;

public class VisualObjectMediatorComponent extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<VisualObject, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				PerceptBelief.class, new VisualObjectTransferFunction(this));
	}

}
