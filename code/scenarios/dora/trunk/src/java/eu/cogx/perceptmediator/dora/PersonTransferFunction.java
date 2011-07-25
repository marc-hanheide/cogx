/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.Person;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class PersonTransferFunction extends
		SimpleDiscreteTransferFunction<Person, GroundedBelief> {

	public PersonTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(PersonTransferFunction.class),
				GroundedBelief.class);

	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, Person from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		result.put("PersonId", PropositionFormula.create(wmc.address.id)
				.getAsFormula());
		result.put("distance", DoubleFormula.create(from.distance)
				.getAsFormula());
		return result;
	}

}
