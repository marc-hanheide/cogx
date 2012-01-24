package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import SpatialProperties.PlaceProperty;

public class PlacePropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		PlaceProperty gpp = (PlaceProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("place ID=" + gpp.placeId);
		// The lines below cause crash if the distribution is empty!
/*		if (gpp.distribution != null
				&& ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value instanceof FloatValue) {

			double val = ((FloatValue) ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value).value;
			extraInfo.add("val=" + val);
		} */
		return extraInfo;
	}

}
