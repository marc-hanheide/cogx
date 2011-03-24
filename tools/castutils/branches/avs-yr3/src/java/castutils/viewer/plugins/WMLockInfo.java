package castutils.viewer.plugins;

import java.util.Vector;

import castutils.slice.WMMutex;


public class WMLockInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		if (iceObject instanceof WMMutex) {
			WMMutex m = (WMMutex) iceObject;
			extraInfo.add("name: " + m.name);
			extraInfo.add("addr: "+m.addr);
		}
		return extraInfo;
	}

}
