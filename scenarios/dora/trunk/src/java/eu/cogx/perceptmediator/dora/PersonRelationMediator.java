package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class PersonRelationMediator extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public PersonRelationMediator() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<Person, GroundedBelief> getMediator(
			String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, Person.class,
				GroundedBelief.class, new PersonRelationTransferFunction(this,
						this.allBeliefs));
	}

}
