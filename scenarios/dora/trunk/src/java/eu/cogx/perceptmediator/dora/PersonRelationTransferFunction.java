/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import org.apache.log4j.Logger;

import VisionData.Person;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.LocalizedRelationTransferFunction;

/**
 * @author marc
 * 
 */
public class PersonRelationTransferFunction extends
		LocalizedRelationTransferFunction<Person, GroundedBelief> {

	public PersonRelationTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(PersonRelationTransferFunction.class),
				GroundedBelief.class);
	}

	@Override
	protected ContentMatchingFunction<GroundedBelief> getMatchingFunction(
			String id) {
		return new PersonMatchingFunction(id);
	}

}
