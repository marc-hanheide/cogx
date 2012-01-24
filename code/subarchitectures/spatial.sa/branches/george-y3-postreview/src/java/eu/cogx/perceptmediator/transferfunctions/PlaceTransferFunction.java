/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place, GroundedBelief> {

	public static final String PLACE_ID_ID = "PlaceId";
	public static final String PLACE_STATUS_ID = "placestatus";

	public PlaceTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(PlaceTransferFunction.class), GroundedBelief.class);
	}

	@Override
	protected
	Map<String, Formula> getFeatureValueMapping(WorkingMemoryChange wmc, Place from) throws BeliefException {
		assert(from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		result.put(PLACE_ID_ID, IntFormula.create((int) from.id).getAsFormula());
//		BoolFormula isExplored = BoolFormula.create(from.status==PlaceStatus.TRUEPLACE);
		result.put(PLACE_STATUS_ID, PropositionFormula.create(from.status.name()).getAsFormula());
//		result.put("explored", isExplored.getAsFormula());
		return result;
	}


}
