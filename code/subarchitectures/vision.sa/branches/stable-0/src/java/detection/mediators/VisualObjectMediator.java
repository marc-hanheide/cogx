/**
 * 
 */
package detection.mediators;

import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.StringValue;
import VisionData.VisualObject;

/**
 * @author marc
 *
 */
public class VisualObjectMediator extends AbstractDetectedObjectMediator<VisualObject> {

	public VisualObjectMediator() {
		super(VisualObject.class);
	}

	@Override
	protected boolean transform(PlaceContainmentObjectProperty pcop, VisualObject object) {
		pcop.mapValue=new StringValue(object.label);
		// TODO: decide when to add to propagate this observation, assuming always here!
		return true;
	}

}
