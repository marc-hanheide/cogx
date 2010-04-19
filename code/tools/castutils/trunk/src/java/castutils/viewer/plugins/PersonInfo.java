package castutils.viewer.plugins;

import java.util.Vector;

import Ice.ObjectImpl;

public class PersonInfo implements Plugin {

	@Override
	public Vector<Object> toVector(ObjectImpl _iceObject) {
		VisionData.Person p = (VisionData.Person) _iceObject;
		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("distance: " + p.distance);
		extraInfo.add("angle: " + p.angle);
		return extraInfo;
	}

}
