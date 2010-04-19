package binder.perceptmediator.components;

import Ice.ObjectImpl;
import VisionData.VisualObject;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.PerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.VisualObjectTransferFunction;

public class VisualObjectMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				new VisualObjectTransferFunction(this));
	}

}
