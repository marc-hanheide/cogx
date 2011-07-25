/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import org.jaxen.function.StringFunction;

import VisionData.Person;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.slice.lf.Proposition;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class PersonMatchingFunction implements
		ContentMatchingFunction<GroundedBelief> {
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
	public boolean matches(GroundedBelief r) {
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
