package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class PersonMediator extends
		ReferringPerceptMediatorComponent<PerceptBelief> {

	public PersonMediator() {
		super(PerceptBelief.class);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected PerceptBindingMediator<Person, PerceptBelief> getMediator(
			String _toSA) {
		PersonTransferFunction tf = new PersonTransferFunction(this, allBeliefs);
		return PerceptBindingMediator.create(this, _toSA, Person.class,
				PerceptBelief.class, tf);
		//
		// return PerceptBindingMediator.create(this, Person.class,
		// new LocalizedPersonTransferFunction(this, perceptBeliefsView),
		// );
	}

}
