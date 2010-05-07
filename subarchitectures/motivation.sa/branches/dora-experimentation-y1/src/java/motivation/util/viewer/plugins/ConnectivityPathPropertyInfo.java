package motivation.util.viewer.plugins;

import java.util.Vector;

import SpatialProperties.ConnectivityPathProperty;

public class ConnectivityPathPropertyInfo implements Plugin {
	
	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ConnectivityPathProperty cpp = (ConnectivityPathProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("place1ID=" + cpp.place1Id);
		extraInfo.add("place2ID=" + cpp.place2Id);
		return extraInfo;
	}

}
