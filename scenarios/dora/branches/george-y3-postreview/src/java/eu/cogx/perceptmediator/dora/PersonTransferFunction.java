/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import spatial.execution.TurnToPersonExecutor;

import SpatialData.Place;
import VisionData.Person;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentLinkingDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import execution.slice.person.PersonObservation;

/**
 * @author marc
 * 
 */
public class PersonTransferFunction
		extends
		DependentLinkingDiscreteTransferFunction<PersonObservation, PerceptBelief, GroundedBelief> {

	public static final String ATTR_POS_THETA = "posTheta";
	public static final String ATTR_POS_Y = "posY";
	public static final String ATTR_POS_X = "posX";
	private static final double PROB_EPS = 0.001;
	public static final String EXISTS = "does-exist";
	public static final String IS_IN = "is-in";
	public static final String PERSON_ID = "PersonId";

	public PersonTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {

		super(component, allBeliefs, Logger
				.getLogger(PersonTransferFunction.class), PerceptBelief.class);

	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, PersonObservation from)
			throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		if (from.identifier.equals("commander")) {
			println("the 'commander' is automatically engaged");
			result.put(TurnToPersonExecutor.ENGAGED, BoolFormula.create(true).getAsFormula());
			result.put(TurnToPersonExecutor.UNRESPONSIVE, BoolFormula.create(false).getAsFormula());
		}
		result.put(PERSON_ID, PropositionFormula.create(from.identifier)
				.getAsFormula());
		// result.put("distance", DoubleFormula.create(from.distance)
		// .getAsFormula());
		try {
			WorkingMemoryAddress robotBelAddr = getReferredBelief(new WMContentWaiter.ContentMatchingFunction<GroundedBelief>() {

				@Override
				public boolean matches(GroundedBelief viewContent) {
					return viewContent.type.equals("Robot");
				}
			});
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> robotBel = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, component.getMemoryEntry(
							robotBelAddr, GroundedBelief.class));
			PointerFormula isIn = (PointerFormula) robotBel.getContent().get(
					LocalizedAgentTransferFunction.IS_IN).getDistribution()
					.getMostLikely().get();

			WorkingMemoryAddress placeBel = isIn.pointer;
			WMPointer ptr = WMPointer.create(placeBel, CASTUtils
					.typeName(Place.class));
			result.put(IS_IN, ptr.getAsFormula());
			result
					.put(ATTR_POS_X, DoubleFormula.create(from.robotX)
							.getAsFormula());
			result
					.put(ATTR_POS_Y, DoubleFormula.create(from.robotY)
							.getAsFormula());
			result.put(ATTR_POS_THETA, DoubleFormula.create(from.robotTheta)
					.getAsFormula());
			
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return result;
	}

	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> belief,
			WorkingMemoryChange wmc, PersonObservation from) {

		super.fillBelief(belief, wmc, from);
		belief.setType(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(Person.class));
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(true, (from.existProb > PROB_EPS) ? 1.0 : 0.0);
		belief.getContent().put(EXISTS, fd);
	}

}
