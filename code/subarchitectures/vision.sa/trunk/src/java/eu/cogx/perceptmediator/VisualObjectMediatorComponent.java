package eu.cogx.perceptmediator;

import VisionData.VisualObject;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class VisualObjectMediatorComponent extends
		ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<VisualObject> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				new LocalizedObjectTransferFunction(this, perceptBeliefsView));
	}

}
