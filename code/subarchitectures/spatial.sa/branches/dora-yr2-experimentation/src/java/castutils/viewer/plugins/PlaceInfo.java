package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import SpatialData.Place;


public class PlaceInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Place m = (Place) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add(m.id);
		extraInfo.add(m.status.name());
		return extraInfo;
	}

}
