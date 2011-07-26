package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringBeliefMediatorComponent;

public class PersonMediator extends
		ReferringBeliefMediatorComponent<PerceptBelief, GroundedBelief> {

	public PersonMediator() {
		super(GroundedBelief.class);
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
