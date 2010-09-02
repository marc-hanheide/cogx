package eu.cogx.perceptmediator.george;

import VisionData.VisualObject;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;

public class VisualObjectMediatorComponent extends
		PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<VisualObject> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				new VisualObjectTransferFunction(this));
	}

}
