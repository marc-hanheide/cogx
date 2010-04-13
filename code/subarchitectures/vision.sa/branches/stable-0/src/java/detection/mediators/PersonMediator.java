/**
 * 
 */
package detection.mediators;

import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.StringValue;
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
		pcop.mapValue = new StringValue("Person");
		// TODO: check if this makes sense: we only propagate persons if they
		// are closer than a distance
		return object.distance < 1.5;
	}

}
