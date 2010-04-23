package binder.perceptmediator.components;

import Ice.ObjectImpl;
import VisionData.VisualObject;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.VisualObjectRelationTransferFunction;

public class VisualObjectRelationMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				new VisualObjectRelationTransferFunction(this, perceptBeliefsView));
	}

}
