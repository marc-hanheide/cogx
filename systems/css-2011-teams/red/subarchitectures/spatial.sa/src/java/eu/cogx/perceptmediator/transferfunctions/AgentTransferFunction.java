/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.PlaceContainmentAgentProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 *
 */
public class AgentTransferFunction extends SimpleDiscreteTransferFunction<PlaceContainmentAgentProperty, GroundedBelief> {

	public AgentTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(AgentTransferFunction.class), GroundedBelief.class);
		// TODO Auto-generated constructor stub
	}

	/* (non-Javadoc)
	 * @see binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public GroundedBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from) {
		// TODO Auto-generated method stub
		GroundedBelief pb = super.create(idToCreate, wmc, from);
		pb.type="Robot";
		return pb;
	}

	@Override
	protected
	Map<String, Formula> getFeatureValueMapping(WorkingMemoryChange wmc,  PlaceContainmentAgentProperty from) throws BeliefException {
		assert(from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		result.put("AgentId", IntFormula.create((int) from.agentID).getAsFormula());
		return result;
	}


}
