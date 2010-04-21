package binder.perceptmediator.components;

import java.util.EnumSet;

import Ice.ObjectImpl;
import VisionData.Person;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.LocalizedPersonTransferFunction;
import cast.cdl.WorkingMemoryOperation;

public class PersonMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, Person.class,
				new LocalizedPersonTransferFunction(this, perceptBeliefsView), EnumSet.of(
						WorkingMemoryOperation.ADD,
						WorkingMemoryOperation.OVERWRITE));
	}

}
