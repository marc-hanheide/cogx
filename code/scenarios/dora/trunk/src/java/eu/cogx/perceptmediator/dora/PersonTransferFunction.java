/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import VisionData.Person;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public class PersonTransferFunction extends
		DependentDiscreteTransferFunction<Person, PerceptBelief> {

	public static final String IS_IN = "is-in";
	public static final String PERSON_ID = "PersonId";

	public PersonTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(PersonTransferFunction.class), PerceptBelief.class);

	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, Person from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		result.put(PERSON_ID, PropositionFormula.create(wmc.address.id)
				.getAsFormula());
		result.put("distance", DoubleFormula.create(from.distance)
				.getAsFormula());
		try {
			Place place = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeBel = getReferredBelief(new PlaceMatchingFunction(
					place.id));
			WMPointer ptr = WMPointer.create(placeBel, CASTUtils
					.typeName(Place.class));
			result.put(IS_IN, ptr.getAsFormula());
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return result;
	}

}
