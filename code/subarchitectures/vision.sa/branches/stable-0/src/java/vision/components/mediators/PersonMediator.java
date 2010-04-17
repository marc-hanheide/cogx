/**
 * 
 */
package vision.components.mediators;

import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import SpatialProperties.PlaceContainmentObjectProperty;
import VisionData.Person;

/**
 * @author marc
 * 
 */
public class PersonMediator extends AbstractDetectedObjectMediator<Person> {

	static int counter=0;
	
	public PersonMediator() {
		super(Person.class, false);
	}

	@Override
	protected boolean transform(PlaceContainmentObjectProperty pcop,
			Person object, WorkingMemoryChange event) {
		pcop.label="Person";
		if (event.operation==WorkingMemoryOperation.ADD)
			pcop.objectId = counter++;
		// TODO: check if this makes sense: we only propagate persons if they
		// are closer than a distance
		return object.distance < 1.5;
	}

}
