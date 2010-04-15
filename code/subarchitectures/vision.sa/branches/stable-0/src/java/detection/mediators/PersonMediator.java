/**
 * 
 */
package detection.mediators;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.StringValue;
import SpatialProperties.ValueProbabilityPair;
import VisionData.Person;

/**
 * @author marc
 * 
 */
public class PersonMediator extends AbstractDetectedObjectMediator<Person> {

	public PersonMediator() {
		super(Person.class);
	}

	@Override
	protected boolean transform(PlaceContainmentObjectProperty pcop,
			Person object) {
		ValueProbabilityPair[] data = new ValueProbabilityPair[1];
		data[0]=new ValueProbabilityPair(new StringValue("Person"), 1.0);
		pcop.distribution = new DiscreteProbabilityDistribution(data);
		// TODO: check if this makes sense: we only propagate persons if they
		// are closer than a distance
		return object.distance < 1.5;
	}

}
