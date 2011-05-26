package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import SpatialProperties.GatewayPlaceProperty;

public class GatewayPlacePropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		GatewayPlaceProperty gpp = (GatewayPlaceProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("place ID=" + gpp.placeId);
		return extraInfo;
	}

}
