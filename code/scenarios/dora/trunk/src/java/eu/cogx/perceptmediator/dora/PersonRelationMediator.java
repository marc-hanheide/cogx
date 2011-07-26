package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class PersonRelationMediator extends
		ReferringPerceptMediatorComponent<PerceptBelief> {

	public PersonRelationMediator() {
		super(PerceptBelief.class);
	}

	@Override
	protected PerceptBindingMediator<Person, PerceptBelief> getMediator(
			String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, Person.class,
				PerceptBelief.class, new PersonRelationTransferFunction(this,
						this.allBeliefs));
	}

}
