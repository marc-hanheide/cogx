/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProbabilities.JointProbabilityValue;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class ComaRoomTransferFunction extends
		SimpleDiscreteTransferFunction<ComaRoom, GroundedBelief> {

	public static final String ROOM_ID = "RoomId";
	public static final String CATEGORY_ID = "category";
	private final double hasPersonProbability;

	public ComaRoomTransferFunction(ManagedComponent component,
			double hasPersonProbability) {
		super(component, Logger.getLogger(ComaRoomTransferFunction.class),
				GroundedBelief.class);
		this.hasPersonProbability = hasPersonProbability;
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ComaRoom from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		result
				.put(ROOM_ID, IntFormula.create((int) from.roomId)
						.getAsFormula());

		// BoolFormula isExplored =
		// BoolFormula.create(from.status==PlaceStatus.TRUEPLACE);
		// result.put("placestatus",
		// PropositionFormula.create(from.status.name()).getAsFormula());
		// result.put("explored", isExplored.getAsFormula());
		return result;
	}

	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			WorkingMemoryChange wmc, ComaRoom from) {
		super.fillBelief(belief, wmc, from);
		if (from.categories.massFunction == null) {
			component.getLogger().info(
					"Coma room without a category yet, not mediating!");
			return;
		}
		// logger.info("fill belief with coma categories");
		IndependentFormulaDistributions distr = belief.getContent();
		FormulaDistribution fd = FormulaDistribution.create();
		for (JointProbabilityValue jp : from.categories.massFunction) {
			String value = ((SpatialProbabilities.StringRandomVariableValue) (jp.variableValues[0])).value;
			// logger.info("adding " + value + " (" + jp.probability + ")");
			fd.add(value, jp.probability);
		}
		// assert (fd.size() > 0);
		distr.put(CATEGORY_ID, fd);

		FormulaDistribution fd_person = FormulaDistribution.create();
		fd_person.add(true, hasPersonProbability);
		distr.put("contains-a-person-prior", fd_person);
	}

}
