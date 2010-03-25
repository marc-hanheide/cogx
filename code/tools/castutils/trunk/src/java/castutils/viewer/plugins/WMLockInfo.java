package motivation.util.viewer.plugins;

import java.util.Vector;

import motivation.slice.WMMutex;
import autogen.Planner.Action;

public class WMLockInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		if (iceObject instanceof Action) {
			WMMutex m = (WMMutex) iceObject;
			extraInfo.add("name: " + m.name);
			extraInfo.add("addr: "+m.addr);
		}
		return extraInfo;
	}

}
