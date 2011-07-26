/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import VisionData.Person;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class PersonMatchingFunction<T extends dBelief> implements
		ContentMatchingFunction<T> {
	/** the identifier of the PlaceId attribute */
	public static final String PLACE_ID = "PlaceId";

	private String personId;

	/**
	 * @param personId
	 */
	public PersonMatchingFunction(String personId) {
		this.personId = personId;
	}

	@Override
	public boolean matches(T r) {
		if (r.type.equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(Person.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
					.create(dBelief.class, r);
			FormulaDistribution fv = b.getContent().get("PersonId");
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			return mostLikelyPlace.getProposition().equals(personId);
		} else {
			return false;
		}
	}

}
