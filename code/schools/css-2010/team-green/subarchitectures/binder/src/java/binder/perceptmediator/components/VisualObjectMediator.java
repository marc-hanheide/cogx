package binder.perceptmediator.components;

import java.util.EnumSet;

import Ice.ObjectImpl;
import VisionData.VisualObject;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.LocalizedVisualObjectTransferFunction;
import cast.cdl.WorkingMemoryOperation;

public class VisualObjectMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {


		return PerceptBindingMediator.create(this, VisualObject.class,
				new LocalizedVisualObjectTransferFunction(this, perceptBeliefsView), EnumSet.of(
						WorkingMemoryOperation.ADD,
						WorkingMemoryOperation.OVERWRITE));
	}

}
