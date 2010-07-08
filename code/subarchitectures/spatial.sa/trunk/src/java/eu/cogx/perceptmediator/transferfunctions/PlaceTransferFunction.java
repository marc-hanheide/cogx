/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place> {

	public PlaceTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(PlaceTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, Formula> getFeatureValueMapping(WorkingMemoryChange wmc, Place from) throws BeliefException {
		assert(from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		result.put("PlaceId", IntFormula.create((int) from.id).getAsFormula());
		BoolFormula isExplored = BoolFormula.create(from.status==PlaceStatus.TRUEPLACE);
		result.put("status", PropositionFormula.create(from.status.name()).getAsFormula());
		result.put("explored", isExplored.getAsFormula());
		return result;
	}


}
