package eu.cogx.perceptmediator.dora;

import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringBeliefMediatorComponent;
import execution.slice.person.PersonObservation;

public class PersonMediator extends
		ReferringBeliefMediatorComponent<PerceptBelief, GroundedBelief> {

	public PersonMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<PersonObservation, PerceptBelief> getMediator(
			String _toSA) {
		PersonTransferFunction tf = new PersonTransferFunction(this, allBeliefs);

		return PerceptBindingMediator.create(this, _toSA, PersonObservation.class,
				PerceptBelief.class, tf);
		//
		// return PerceptBindingMediator.create(this, Person.class,
		// new LocalizedPersonTransferFunction(this, perceptBeliefsView),
		// );
	}

}
