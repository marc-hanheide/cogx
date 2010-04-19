/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.Person;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 *
 */
public class PersonTransferFunction extends SimpleDiscreteTransferFunction<Person> {

	public PersonTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(PersonTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc,  Person from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		result.put("PersonId", FeatureValueBuilder.createNewStringValue(wmc.address.id));
		result.put("distance", FeatureValueBuilder.createNewFloatValue(from.distance));
		return result;
	}


}
