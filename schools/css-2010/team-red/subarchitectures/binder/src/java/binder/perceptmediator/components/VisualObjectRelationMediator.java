package binder.perceptmediator.components;

import java.util.EnumSet;

import cast.cdl.WorkingMemoryOperation;
import Ice.ObjectImpl;
import VisionData.VisualObject;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.VisualObjectRelationTransferFunction;

public class VisualObjectRelationMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, VisualObject.class,
				new VisualObjectRelationTransferFunction(this, perceptBeliefsView),
				EnumSet.of(WorkingMemoryOperation.ADD,
						WorkingMemoryOperation.OVERWRITE));
	}

}
