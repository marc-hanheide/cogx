/**
 * 
 */
package detection.mediators;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.StringValue;
import SpatialProperties.ValueProbabilityPair;
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
		ValueProbabilityPair[] data = new ValueProbabilityPair[1];
		data[0]=new ValueProbabilityPair(new StringValue(object.label), 1.0);
		pcop.distribution = new DiscreteProbabilityDistribution(data);
		// TODO: decide when to add to propagate this observation, assuming always here!
		return true;
	}

}
