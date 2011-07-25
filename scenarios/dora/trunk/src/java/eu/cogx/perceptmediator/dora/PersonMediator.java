package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class PersonMediator extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public PersonMediator() {
		super(GroundedBelief.class);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected PerceptBindingMediator<Person, GroundedBelief> getMediator(
			String _toSA) {
		PersonTransferFunction tf = new PersonTransferFunction(this);
		return PerceptBindingMediator.create(this, _toSA, Person.class,
				GroundedBelief.class, tf);
		//
		// return PerceptBindingMediator.create(this, Person.class,
		// new LocalizedPersonTransferFunction(this, perceptBeliefsView),
		// );
	}

}
