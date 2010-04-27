/**
 * 
 */
package vision.components.mediators;

import java.util.EnumSet;
import java.util.Vector;

import vision.components.mediators.abstr.AbstractLocalizedPerceptionMediator;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.ValueProbabilityPair;
import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class VisualObjectMediator extends
		AbstractLocalizedPerceptionMediator<VisualObject, PlaceContainmentObjectProperty> {

	public VisualObjectMediator(ManagedComponent c) {
		super(c, VisualObject.class, PlaceContainmentObjectProperty.class, EnumSet.of(
				WorkingMemoryOperation.ADD, WorkingMemoryOperation.OVERWRITE));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected boolean convert(WorkingMemoryChange wmc, VisualObject from,
			PlaceContainmentObjectProperty to, long placeId) {
		Vector<ValueProbabilityPair> data = new Vector<ValueProbabilityPair>(1);
		to.mapValue = new IntegerValue(placeId);
		data.add(new ValueProbabilityPair(to.mapValue, 1.0));
		to.distribution = new DiscreteProbabilityDistribution(data);
		to.mapValueReliable = true;
		return true;
	}

	@Override
	public PlaceContainmentObjectProperty create(
			WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			VisualObject from) {
		return new PlaceContainmentObjectProperty(from.label.hashCode(), null, null, false, from.label);
	}

}
