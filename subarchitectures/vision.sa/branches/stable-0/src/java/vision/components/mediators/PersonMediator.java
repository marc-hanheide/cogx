/**
 * 
 */
package vision.components.mediators;

import java.util.EnumSet;
import java.util.Vector;

import vision.components.mediators.abstr.AbstractLocalizedPerceptionMediator;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import SpatialProperties.ValueProbabilityPair;
import VisionData.Person;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class PersonMediator extends
		AbstractLocalizedPerceptionMediator<Person, PlaceContainmentAgentProperty> {

	public PersonMediator(ManagedComponent c) {
		super(c, Person.class, PlaceContainmentAgentProperty.class, EnumSet.of(
				WorkingMemoryOperation.ADD, WorkingMemoryOperation.OVERWRITE));
		// TODO Auto-generated constructor stub
	}

	private static long personIdCounter = 1;


	@Override
	protected boolean convert(WorkingMemoryChange wmc, Person from,
			PlaceContainmentAgentProperty to, long placeId) {
		Vector<ValueProbabilityPair> data = new Vector<ValueProbabilityPair>(1);
		to.mapValue = new IntegerValue(placeId);
		data.add(new ValueProbabilityPair(to.mapValue, 1.0));
		to.distribution = new DiscreteProbabilityDistribution(data);
		to.mapValueReliable = true;
		return true;
	}

	@Override
	public PlaceContainmentAgentProperty create(
			WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			Person from) {
		return new PlaceContainmentAgentProperty(personIdCounter++, null, null,
				false);
	}

}
