/**
 * 
 */
package vision.components.mediators;

import SpatialProperties.PlaceContainmentObjectProperty;
import VisionData.VisualObject;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class VisualObjectMediator extends
		AbstractDetectedObjectMediator<VisualObject> {

	/**
	 * this is a threshold to indentify which confidence is required to assume
	 * an object to be detected
	 */
	private static final double CONFIDENCE_THRESHOLD = 0.5;

	public VisualObjectMediator() {
		super(VisualObject.class, false);
	}

	@Override
	protected boolean transform(PlaceContainmentObjectProperty pcop,
			VisualObject object, WorkingMemoryChange event) {
		println("confidence=" + object.detectionConfidence);
		if (object.detectionConfidence < CONFIDENCE_THRESHOLD)
			return false;
		pcop.label = object.label;

		// TODO: using the hashcode assume that each object only exists once!
		pcop.objectId = pcop.label.hashCode();

		return true;
	}

}
