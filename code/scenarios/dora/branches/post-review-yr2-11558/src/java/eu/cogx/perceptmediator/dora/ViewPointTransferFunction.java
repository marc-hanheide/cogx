/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.ViewPoint;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author marc
 * 
 */
public class ViewPointTransferFunction extends
		DependentDiscreteTransferFunction<ViewPoint, GroundedBelief> {

	public static final String OBJECT_PROBABILITY_ID = "probability";
	public static final String OBJECT_LABEL_ID = "label";

	public ViewPointTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(ViewPointTransferFunction.class),
				GroundedBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ViewPoint from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

		// the object which can be found from this cone
		result.put(OBJECT_LABEL_ID, PropositionFormula.create(from.label)
				.getAsFormula());

		// the
		result.put(OBJECT_PROBABILITY_ID, DoubleFormula
				.create(from.probability).getAsFormula());

		int closestPlace = from.closestPlaceId;
		try {
			WorkingMemoryAddress placeBelAdr = getReferredBelief(new PlaceMatchingFunction(
					closestPlace));
			getLogger().debug("looking for place with belief address " + CASTUtils.toString(placeBelAdr));
			GroundedBelief placeBelRaw = allBeliefs.get(placeBelAdr);
			getLogger().debug("looking up the belief returned " + placeBelRaw);
			CASTIndependentFormulaDistributionsBelief<dBelief> placeBel = CASTIndependentFormulaDistributionsBelief
					.create(dBelief.class, placeBelRaw);
			getLogger().info("  placeBel=" + placeBel.toString());
			result.put(RoomMembershipMediator.ROOM_PROPERTY, placeBel.getContent().get(RoomMembershipMediator.ROOM_PROPERTY).getDistribution().getMostLikely());
			getLogger().info("  roommember=" + placeBel.getContent().get(RoomMembershipMediator.ROOM_PROPERTY).getDistribution().getMostLikely().toString());
			result.put(LocalizedAgentTransferFunction.IS_IN, WMPointer.create(placeBelAdr, CASTUtils.typeName(this.beliefClass)).getAsFormula());
//			getLogger().info("  generate for view cone: " + IceXMLSerializer.toXMLString(result));
		} catch (InterruptedException e) {
			getLogger().error(e);
			return null;
		} catch (NullPointerException e) {
			getLogger().error(e);
			return null;
			
		}

		return result;

	}

}
