package binder.perceptmediator.components;

import java.util.EnumSet;

import cast.cdl.WorkingMemoryOperation;

import Ice.ObjectImpl;
import VisionData.Person;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.PerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.PersonTransferFunction;

public class PersonMediator extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, Person.class,
				new PersonTransferFunction(this), EnumSet.of(
						WorkingMemoryOperation.ADD,
						WorkingMemoryOperation.OVERWRITE));
	}

}
