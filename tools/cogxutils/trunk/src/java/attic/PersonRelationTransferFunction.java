/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import org.apache.log4j.Logger;

import VisionData.Person;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.perceptmediator.attic.PersonMatchingFunction;
import binder.perceptmediator.transferfunctions.abstr.LocalizedRelationTransferFunction;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 * 
 */
public class PersonRelationTransferFunction extends
		LocalizedRelationTransferFunction<Person> {

	public PersonRelationTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(PersonRelationTransferFunction.class));
	}

	@Override
	protected ContentMatchingFunction<PerceptBelief> getMatchingFunction(
			String id) {
		return new PersonMatchingFunction(id);
	}

}
