package binder.perceptmediator.components;

import java.util.EnumSet;

import cast.cdl.WorkingMemoryOperation;
import Ice.ObjectImpl;
import VisionData.Person;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import binder.perceptmediator.transferfunctions.PersonRelationTransferFunction;

public class PersonRelationMediator extends ReferringPerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<? extends ObjectImpl> getMediator() {
		return PerceptBindingMediator.create(this, Person.class,
				new PersonRelationTransferFunction(this, perceptBeliefsView),
				EnumSet.of(WorkingMemoryOperation.ADD,
						WorkingMemoryOperation.OVERWRITE));
	}

}
