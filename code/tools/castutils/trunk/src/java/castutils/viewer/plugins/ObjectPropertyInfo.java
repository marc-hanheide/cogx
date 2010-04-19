package castutils.viewer.plugins;

import java.util.Vector;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import SpatialProperties.IntegerValue;
import SpatialProperties.ObjectProperty;
import SpatialProperties.PlaceProperty;

public class ObjectPropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ObjectProperty gpp = (ObjectProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("type=" + gpp.getClass().getSimpleName());
		extraInfo.add("object ID=" + gpp.objectId);
		if (gpp.distribution != null
				&& ((DiscreteProbabilityDistribution) gpp.distribution).data.get(0).value instanceof IntegerValue) {

			long val = ((IntegerValue) ((DiscreteProbabilityDistribution) gpp.distribution).data.get(0).value).value;
			extraInfo.add("val=" + val);
		}
		return extraInfo;
	}

}
