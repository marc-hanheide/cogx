/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import org.apache.log4j.Logger;

import VisionData.Person;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.LocalizedRelationTransferFunction;

/**
 * @author marc
 * 
 */
public class PersonRelationTransferFunction extends
		LocalizedRelationTransferFunction<Person, PerceptBelief> {

	public PersonRelationTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(PersonRelationTransferFunction.class),
				PerceptBelief.class);
	}

	@Override
	protected ContentMatchingFunction<PerceptBelief> getMatchingFunction(
			String id) {
		return new PersonMatchingFunction<PerceptBelief>(id);
	}

}
