package castutils.viewer.plugins;

import java.util.Vector;

import SpatialProperties.ConnectivityPathProperty;

public class ConnectivityPathPropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ConnectivityPathProperty gpp = (ConnectivityPathProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("from ID=" + gpp.place1Id + "to ID=" + gpp.place2Id);
		return extraInfo;
	}

}
