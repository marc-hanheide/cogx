/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.PlaceContainmentAgentProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 *
 */
public class AgentTransferFunction extends SimpleDiscreteTransferFunction<PlaceContainmentAgentProperty> {

	public AgentTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(AgentTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc,  PlaceContainmentAgentProperty from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		result.put("AgentId", FeatureValueBuilder.createNewIntegerValue((int) from.agentID));
		return result;
	}

	/* (non-Javadoc)
	 * @see binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from) {
		// TODO Auto-generated method stub
		PerceptBelief pb = super.create(idToCreate, wmc, from);
		pb.type="Robot";
		return pb;
	}


}
