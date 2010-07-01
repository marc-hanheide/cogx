package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.IntegerValue;
import SpatialProperties.ObjectProperty;

public class ObjectPropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ObjectProperty gpp = (ObjectProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("type=" + gpp.getClass().getSimpleName());
		extraInfo.add("object ID=" + gpp.objectId);
		if (gpp.distribution != null
				&& ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value instanceof IntegerValue) {

			long val = ((IntegerValue) ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value).value;
			extraInfo.add("val=" + val);
		}
		return extraInfo;
	}

}
